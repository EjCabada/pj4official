obj-m += virtdev.o

all:
	make -C /lib/modules/$(shell uname -r)/build/ M=$(PWD) modules

clean:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) clean

user:
	gcc virtdev-user.c -o virtdev-user

install:
	insmod virtdev.ko

remove: 
	rmmod virtdev
