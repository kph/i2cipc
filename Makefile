ifneq ($(DH_VERBOSE),)
	V ?= 1
endif

ifneq ($(V),)
	MAKEFLAGS += V=$(V)
else
	Q = @
	MAKEFLAGS += --no-print-directory
endif

ifneq ($(KDIR),)
	MAKEFLAGS += KDIR=$(KDIR)
endif

# subject, verb, object(s)
svo = $(if $(Q),$(info $1 $3))$(Q)$2 $3

default:

bindeb-pkg:
	$(call svo, DEBUILD,\
		debuild -uc -us -ui -b --lintian-opts --profile debian)

clean: files = $(wildcard README.html)

clean:
	$(call svo, DKMS, $(MAKE) -C src/controller, $@)
	$(call svo, DKMS, $(MAKE) -C src/responder, $@)
	$(if $(files),$(call svo, CLEAN, rm, $(files)))

docs: README.html

modules modules_install:
	$(call svo, DKMS, $(MAKE) -C src/controller, $@)
	$(call svo, DKMS, $(MAKE) -C src/responder, $@)

.PHONY: default bindeb-pkg clean docs modules modules_install

%.html: %.md ; pandoc --from gfm --to html -o $@ $<

FORCE: ;
