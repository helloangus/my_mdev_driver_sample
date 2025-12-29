/* host_mdev_sched.c
 * New-API mdev driver but exposes per-mdev sysfs "in" attribute so users can
 * write to the virtual device (similar to old mdev_parent_ops->write).
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/mdev.h>
#include <linux/slab.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/mutex.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/kfifo.h>
#include <linux/wait.h>
#include <linux/list.h>
#include <linux/device.h>
#include <linux/vfio.h>
#include <linux/timekeeping.h>

#include <linux/platform_device.h>

static struct platform_device *mdev_pdev;


#define DEVICE_NAME "mdev_dsched"
#define LOG_DIR "/var/log"
#define LOG_PREFIX "mdev_dsched"

#define QUEUE_SIZE 4096

struct my_mdev {
    struct mdev_device *mdev;      /* mdev core 对象 */
    struct kfifo queue;            /* 待写入队列 (bytes) */
    unsigned int priority;
    unsigned int timeslice_ms;
    bool running;                  /* 是否当前持有时间片 */
    struct mutex lock;
    struct list_head list_node;    /* 链表连接到父驱动 */

    char uuid[64];
};

struct parent_priv {
    struct mdev_parent parent;     /* 嵌入 mdev_parent */
    struct list_head instances;    /* 链表头，保存 struct my_mdev */
    struct mutex list_lock;
    struct task_struct *sched_thread;
    wait_queue_head_t wake_sched;
    bool stop;

    char log_path[256];
};

static struct parent_priv *g_parent;

/* ========== helper: write kernel buffer to host file ========== */
static int host_write_all(struct parent_priv *p,
                          const char *buf, size_t len)
{
    struct file *filp;
    loff_t pos = 0;
    ssize_t written;

    if (!p || !p->log_path[0])
        return -EINVAL;

    filp = filp_open(p->log_path,
                     O_WRONLY | O_CREAT | O_APPEND,
                     0644);
    if (IS_ERR(filp))
        return PTR_ERR(filp);

    pr_info("%s: writing %zu bytes to log file\n",
            DEVICE_NAME, len);
    written = kernel_write(filp, buf, len, &pos);
    filp_close(filp, NULL);

    return written < 0 ? (int)written : 0;
}


/* ========== scheduler thread ========== */
static int sched_thread_fn(void *data)
{
    struct parent_priv *p = data;

    while (!kthread_should_stop()) {
        struct my_mdev *sel = NULL;
        struct my_mdev *iter;
        unsigned long best_prio = 0;
        bool found = false;

        /* pick the highest priority non-empty instance */
        mutex_lock(&p->list_lock);
        list_for_each_entry(iter, &p->instances, list_node) {
            if (!kfifo_is_empty(&iter->queue)) {
                found = true;
                if (iter->priority >= best_prio) {
                    best_prio = iter->priority;
                    sel = iter;
                }
            }
        }
        mutex_unlock(&p->list_lock);

        if (!found) {
            /* 没有数据，等待唤醒或超时。等待条件：stop 或 instances 不为空（轮询） */
            wait_event_interruptible_timeout(p->wake_sched,
                                             p->stop || !list_empty(&p->instances),
                                             msecs_to_jiffies(200));
            if (p->stop || kthread_should_stop())
                break;
            continue;
        }

        if (!sel)
            continue;

        /* grant timeslice */
        mutex_lock(&sel->lock);
        sel->running = true;

        /* 从队列读数据并写到 host 文件（示例行为） */
        {
            unsigned int avail = kfifo_len(&sel->queue);
            unsigned int tocopy = min(avail, (unsigned int)(QUEUE_SIZE - 1));
            char *buf = kmalloc(tocopy + 2, GFP_KERNEL);
            if (buf) {
                unsigned int copied_bytes = kfifo_out(&sel->queue, buf, tocopy);
                if (copied_bytes > 0) {
                    host_write_all(p, "[", 1);
                    host_write_all(p, sel->uuid, strlen(sel->uuid));
                    host_write_all(p, "] \r\n", 4);
                    host_write_all(p, buf, copied_bytes);
                    host_write_all(p, "\n", 1);
                }
                kfree(buf);
            }
        }

        if (sel->timeslice_ms)
            msleep(sel->timeslice_ms);
        else
            msleep(100);

        sel->running = false;

        mutex_unlock(&sel->lock);

        if (kthread_should_stop() || p->stop)
            break;
    }

    return 0;
}

/* ========== sysfs "in" attribute store function (per-mdev) ==========
 * Writing to /sys/.../<mdev>/in will enqueue data into that mdev's kfifo
 * and wake the scheduler (behaves like old parent->write).
 */
static ssize_t mdev_in_store(struct device *dev,
                             struct device_attribute *attr,
                             const char *buf, size_t count)
{
    struct my_mdev *mm = dev_get_drvdata(dev);
    size_t tocopy;

    if (!mm)
        return -EINVAL;

    /* 限制大小，避免一次放入过多 */
    tocopy = min(count, (size_t)(QUEUE_SIZE - 1));
    /* kfifo_in 接受 kernel buffer */
    tocopy = kfifo_in(&mm->queue, buf, tocopy);

    /* 唤醒调度线程 */
    if (g_parent)
        pr_info("%s: waking scheduler after enqueue %zu bytes to mdev %s\n",
                DEVICE_NAME, tocopy, mm->uuid);
        wake_up_interruptible(&g_parent->wake_sched);

    /* 返回写入的字节（note: sysfs expects count or consumed len; return count to mimic user write) */
    return count;
}

static ssize_t mdev_modify_priority_store(struct device *dev,
                             struct device_attribute *attr,
                             const char *buf, size_t count)
{
    struct my_mdev *mm = dev_get_drvdata(dev);
    unsigned int new_prio;
    int ret;

    if (!mm)
        return -EINVAL;

    ret = kstrtouint(buf, 10, &new_prio);
    if (ret)
        return ret;

    mutex_lock(&mm->lock);
    mm->priority = new_prio;
    mutex_unlock(&mm->lock);

    pr_info("%s: mdev %s priority changed to %u\n", DEVICE_NAME,
            mm->uuid, mm->priority);

    return count;
}

static int my_mdev_create(struct mdev_device *mdev)
{
    struct my_mdev *mm;

    mm = kzalloc(sizeof(*mm), GFP_KERNEL);
    if (!mm)
        return -ENOMEM;

    mm->mdev = mdev;
    if (kfifo_alloc(&mm->queue, QUEUE_SIZE, GFP_KERNEL)) {
        kfree(mm);
        return -ENOMEM;
    }
    mm->priority = 1;
    mm->timeslice_ms = 100;
    mutex_init(&mm->lock);
    INIT_LIST_HEAD(&mm->list_node);

    strscpy(mm->uuid,
        dev_name(mdev_dev(mdev)),
        sizeof(mm->uuid));

    /* attach mm 到 mdev->priv（使用 mdev_dev 宏转换到 struct device）*/
    dev_set_drvdata(mdev_dev(mdev), mm);

    /* add 到父驱动实例链表（如果有） */
    if (g_parent) {
        mutex_lock(&g_parent->list_lock);
        list_add_tail(&mm->list_node, &g_parent->instances);
        mutex_unlock(&g_parent->list_lock);
        wake_up_interruptible(&g_parent->wake_sched);
    }

    return 0;
}


static void my_mdev_remove(struct mdev_device *mdev)
{
    struct my_mdev *mm = dev_get_drvdata(mdev_dev(mdev));
    if (!mm)
        return;

    if (g_parent) {
        mm->running = false;
        mutex_lock(&g_parent->list_lock);
        list_del(&mm->list_node);
        mutex_unlock(&g_parent->list_lock);
    }

    kfifo_free(&mm->queue);
    kfree(mm);
    pr_info("%s: removed mdev instance\n", DEVICE_NAME);
}

/* minimal mdev_type */
static struct mdev_type my_mdev_type = {
    .sysfs_name = "mdev_dsched",
    .pretty_name = "mdev dsched",
};

static struct mdev_type *my_mdev_types[] = {
    &my_mdev_type,
};


static struct mdev_driver my_mdev_driver;

/* We'll use one static device_attribute named "in" (per-device kobj supports same attribute name) */
static DEVICE_ATTR(in, 0220, NULL, mdev_in_store);
static DEVICE_ATTR(priority, 0220, NULL, mdev_modify_priority_store);

// 虚拟设备 driver probe/remove
static int my_mdev_probe(struct mdev_device *mdev)
{
    pr_info("%s: probe mdev=%p dev=%p driver=%p\n", DEVICE_NAME,
        mdev, mdev_dev(mdev), my_mdev_driver.driver);

    int ret;

    /* create instance */
    ret = my_mdev_create(mdev);
    if (ret)
        return ret;

    /* create per-device sysfs "in" attribute */
    /* device_create_file returns 0 on success */
    ret = device_create_file(mdev_dev(mdev), &dev_attr_in);
    if (ret) {
        pr_err("%s: failed to create /sys/.../in (%d)\n", DEVICE_NAME, ret);
        /* cleanup created instance */
        my_mdev_remove(mdev);
        return ret;
    }
    ret = device_create_file(mdev_dev(mdev), &dev_attr_priority);
    if (ret) {
        pr_err("%s: failed to create /sys/.../priority (%d)\n", DEVICE_NAME, ret);
        /* cleanup created instance */
        my_mdev_remove(mdev);
        return ret;
    }

    return 0;
}

static void my_mdev_remove_drv(struct mdev_device *mdev)
{
    pr_info("%s: remove mdev=%p dev=%p driver=%p\n", DEVICE_NAME,
        mdev, mdev_dev(mdev), my_mdev_driver.driver);


    /* remove attribute and instance */
    device_remove_file(mdev_dev(mdev), &dev_attr_in);
    device_remove_file(mdev_dev(mdev), &dev_attr_priority);
    my_mdev_remove(mdev);
}


static struct mdev_driver my_mdev_driver = {
    .device_api = VFIO_DEVICE_API_PCI_STRING,
    .max_instances = 99,
    .probe = my_mdev_probe,
    .remove = my_mdev_remove_drv,
    .get_available = NULL,
    .show_description = NULL,
    .driver = {
        .name  = DEVICE_NAME,
        .owner = THIS_MODULE,
    },
};


// 父设备 driver probe/remove
static int mdev_dsched_pdrv_probe(struct platform_device *pdev)
{
    int ret;
    struct parent_priv *p;

    p = devm_kzalloc(&pdev->dev, sizeof(*p), GFP_KERNEL);
    if (!p)
        return -ENOMEM;

    INIT_LIST_HEAD(&p->instances);
    mutex_init(&p->list_lock);
    init_waitqueue_head(&p->wake_sched);
    p->stop = false;

    p->parent.dev = &pdev->dev;

    {
        struct timespec64 ts;
        struct tm tm;

        ktime_get_real_ts64(&ts);
        time64_to_tm(ts.tv_sec, 0, &tm);

        snprintf(p->log_path, sizeof(p->log_path),
                LOG_DIR "/%s_%04ld%02d%02d_%02d%02d%02d.txt",
                LOG_PREFIX,
                tm.tm_year + 1900,
                tm.tm_mon + 1,
                tm.tm_mday,
                tm.tm_hour,
                tm.tm_min,
                tm.tm_sec);

        dev_info(&pdev->dev, "log file: %s\n", p->log_path);
    }

    ret = mdev_register_parent(&p->parent,
                               &pdev->dev,
                               &my_mdev_driver,
                               my_mdev_types,
                               ARRAY_SIZE(my_mdev_types));
    if (ret) {
        dev_err(&pdev->dev, "mdev_register_parent failed: %d\n", ret);
        return ret;
    }

    p->sched_thread = kthread_run(sched_thread_fn, p, "mdev_sched");
    if (IS_ERR(p->sched_thread)) {
        ret = PTR_ERR(p->sched_thread);
        mdev_unregister_parent(&p->parent);
        return ret;
    }

    platform_set_drvdata(pdev, p);
    g_parent = p;

    dev_info(&pdev->dev, "parent registered and scheduler thread started\n");
    return 0;
}

static int mdev_dsched_pdrv_remove(struct platform_device *pdev)
{
    struct parent_priv *p = platform_get_drvdata(pdev);

    if (!p)
        return 0;

    p->stop = true;
    if (p->sched_thread)
        kthread_stop(p->sched_thread);

    mdev_unregister_parent(&p->parent);
    return 0;
}


static struct platform_driver mdev_dsched_pdrv = {
    .probe  = mdev_dsched_pdrv_probe,
    .remove = mdev_dsched_pdrv_remove,
    .driver = {
        .name = "mdev_dsched_platform_parent",
        .owner = THIS_MODULE,
    },
};



static int __init my_mdev_init(void)
{
    int ret;

    /* 1. register mdev driver FIRST */
    ret = mdev_register_driver(&my_mdev_driver);
    pr_info("mdev_register_driver returned %d\n", ret);
    if (ret)
        return ret;

    /* 2. register platform driver */
    ret = platform_driver_register(&mdev_dsched_pdrv);
    if (ret)
        goto err_unregister_mdev_driver;

    /* 3. create parent device */
    mdev_pdev = platform_device_register_simple("mdev_dsched_platform_parent", -1, NULL, 0);
    if (IS_ERR(mdev_pdev)) {
        ret = PTR_ERR(mdev_pdev);
        goto err_unregister_platform_driver;
    }

    return 0;

err_unregister_platform_driver:
    platform_driver_unregister(&mdev_dsched_pdrv);
err_unregister_mdev_driver:
    mdev_unregister_driver(&my_mdev_driver);
    return ret;
}




static void __exit my_mdev_exit(void)
{
    pr_info("%s: unloading\n", DEVICE_NAME);

    if (mdev_pdev)
        platform_device_unregister(mdev_pdev);

    platform_driver_unregister(&mdev_dsched_pdrv);

    mdev_unregister_driver(&my_mdev_driver);

    pr_info("%s: module unloaded\n", DEVICE_NAME);
}


module_init(my_mdev_init);
module_exit(my_mdev_exit);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Angus Lee");
