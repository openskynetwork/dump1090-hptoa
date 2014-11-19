#
# makefile for piaware doc files
#

DOCDIR=$(DESTDIR)/usr/share/man/man1

MANPAGES= dump1090.1

all:
	@echo "'make install' to install docs"

install:
	@echo ---- installing docs
	install -d $(DOCDIR)
	install -t $(DOCDIR) $(MANPAGES)

