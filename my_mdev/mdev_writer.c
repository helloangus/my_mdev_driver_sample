#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <getopt.h>
#include <time.h>
#include <sys/time.h>

#define SYSFS_FMT "/sys/bus/mdev/devices/%s/in"
#define MAX_BUF   4096

static void usage(const char *prog)
{
    fprintf(stderr,
        "Usage: %s --uuid <uuid> --id <id> [options]\n"
        "Options:\n"
        "  --size <bytes>     bytes per write (default: 64)\n"
        "  --interval <us>    interval between writes in us (default: 100000)\n"
        "  --count <N>        number of writes (0 = infinite)\n",
        prog);
}

int main(int argc, char *argv[])
{
    char uuid[128] = {0};
    char sysfs_path[256];
    char writer_id[64] = "writer";
    size_t write_size = 64;
    unsigned int interval_us = 100000;
    unsigned long count = 0;

    static struct option long_opts[] = {
        {"uuid",     required_argument, 0, 'u'},
        {"id",       required_argument, 0, 'i'},
        {"size",     required_argument, 0, 's'},
        {"interval", required_argument, 0, 't'},
        {"count",    required_argument, 0, 'c'},
        {0, 0, 0, 0}
    };

    int opt;
    while ((opt = getopt_long(argc, argv, "u:i:s:t:c:", long_opts, NULL)) != -1) {
        switch (opt) {
        case 'u':
            strncpy(uuid, optarg, sizeof(uuid) - 1);
            break;
        case 'i':
            strncpy(writer_id, optarg, sizeof(writer_id) - 1);
            break;
        case 's':
            write_size = strtoul(optarg, NULL, 0);
            break;
        case 't':
            interval_us = strtoul(optarg, NULL, 0);
            break;
        case 'c':
            count = strtoul(optarg, NULL, 0);
            break;
        default:
            usage(argv[0]);
            return 1;
        }
    }

    if (uuid[0] == '\0') {
        usage(argv[0]);
        return 1;
    }

    snprintf(sysfs_path, sizeof(sysfs_path),
             SYSFS_FMT, uuid);

    int fd = open(sysfs_path, O_WRONLY);
    if (fd < 0) {
        perror("open sysfs");
        return 1;
    }

    if (write_size > MAX_BUF)
        write_size = MAX_BUF;

    char buf[MAX_BUF];
    unsigned long iter = 0;

    fprintf(stderr,
        "[writer %s] uuid=%s size=%zu interval=%u us count=%lu\n",
        writer_id, uuid, write_size, interval_us, count);

    while (count == 0 || iter < count) {
        struct timeval tv;
        gettimeofday(&tv, NULL);

        int len = snprintf(buf, sizeof(buf),
                           "id=%s seq=%lu ts=%ld.%06ld",
                           writer_id, iter,
                           tv.tv_sec, tv.tv_usec);

        if ((size_t)len < write_size) {
            memset(buf + len, 'X', write_size - len);
            len = write_size;
        }

        ssize_t ret = write(fd, buf, len);
        if (ret < 0) {
            perror("write");
            break;
        }

        iter++;
        if (interval_us)
            usleep(interval_us);
    }

    close(fd);
    return 0;
}
