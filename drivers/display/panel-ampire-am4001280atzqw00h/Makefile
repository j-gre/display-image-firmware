obj-m := panel-ampire-am4001280atzqw00h.o

PWD := $(shell pwd)
.PHONY = check

all:
    $(MAKE) -C $(KERNEL_SRC) M=$(PWD) 

modules_install:
    $(MAKE) -C $(KERNEL_SRC) M=$(PWD) modules_install

clean:
    rm -f *.o *~ core .depend .*.cmd *.ko *.mod.c
    rm -f Module.markers Module.symvers modules.order
    rm -rf .tmp_versions Modules.symvers

cfiles = $(obj-m:.o=.c)
check: $(cfiles)
	$(KERNEL_SRC)/scripts/checkpatch.pl -f --max-line-length=100 $<

