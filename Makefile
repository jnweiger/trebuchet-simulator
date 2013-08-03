NAME:=trebuchet
VERS:=$(shell date +%Y%m%d)
EXCL:=$(NAME)-$(VERS) *.orig *.pyc *.tar*

all:
	@echo just run *.sh

dist:
	mkdir -p $(NAME)-$(VERS); cd $(NAME)-$(VERS); ln -sf ../* .; rm -rf -- $(EXCL)
	tar jcvhf $(NAME)-$(VERS).tar.bz2 $(NAME)-$(VERS)
	rm -rf $(NAME)-$(VERS)
