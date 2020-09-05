make = make -C /lib/modules/$(shell uname -r)/build M=$(shell pwd)

all:
	$(make)

.PHONY: clean tags scp

clean:
	$(make) clean

tags:
	ctags -R . /lib/modules/$(shell uname -r)/build/include

scp:
	scp thing.ko root@jig:
