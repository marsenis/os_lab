#include <stdio.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <string.h>

#define BUF_SZ 100

char buf[BUF_SZ + 5];

int main(int argc, char *argv[]) {
	int fd, i = 1;
	ssize_t ret;
	char *filename;

	if (argc != 2) {
		printf("Usage: %s [dev-file]\n", argv[0]);
		return 0;
	}

	filename = argv[1];
	fd = open(filename, O_RDONLY);

	if (fd < 0) {
		perror("Error while opening file");
		return 1;
	}

	while ( (ret = read(fd, buf, BUF_SZ)) > 0 ) {
		write(0, buf, ret);

		if (ioctl(fd, 2, (i++) % 2) < 0) {
			perror("[ioctl] ");
		}
	}

	if (ret < 0) {
		perror("Error while reading file");
		return 2;
	}

	close(fd);
	return 0;
}
