#
# When building a package or installing otherwise in the system, make
# sure that the variable PREFIX is defined, e.g. make PREFIX=/usr/local
#
PROGNAME=dump1090

include /usr/share/dpkg/buildflags.mk

PREFIX=$(DESTDIR)/usr

BINDIR=$(PREFIX)/bin
SHAREDIR=$(PREFIX)/share/$(PROGNAME)
HTMLDIR=$(SHAREDIR)/public_html
ETCDIR=$(DESTDIR)/etc
EXTRACFLAGS=-DHTMLPATH=\"$(HTMLDIR)\"

#CFLAGS=-O2 -g -Wall -W `pkg-config --cflags librtlsdr`
CFLAGS+= `pkg-config --cflags librtlsdr`
LIBS=`pkg-config --libs librtlsdr` -lpthread -lm
CC=gcc


all: dump1090 view1090 faup1090

%.o: %.c
	$(CC) $(CPPFLAGS) $(CFLAGS) $(EXTRACFLAGS) -c $<

dump1090: dump1090.o anet.o interactive.o mode_ac.o mode_s.o net_io.o
	$(CC) -g -o dump1090 dump1090.o anet.o interactive.o mode_ac.o mode_s.o net_io.o $(LIBS) $(LDFLAGS)

view1090: view1090.o anet.o interactive.o mode_ac.o mode_s.o net_io.o
	$(CC) -g -o view1090 view1090.o anet.o interactive.o mode_ac.o mode_s.o net_io.o $(LIBS) $(LDFLAGS)

faup1090: faup1090.o anet.o interactive.o mode_ac.o mode_s.o net_io.o
	$(CC) -g -o faup1090 faup1090.o anet.o interactive.o mode_ac.o mode_s.o net_io.o $(LIBS) $(LDFLAGS)

# not built by default:
ppup1090: ppup1090.o anet.o interactive.o mode_ac.o mode_s.o net_io.o
	$(CC) -g -o ppup1090 ppup1090.o anet.o interactive.o mode_ac.o mode_s.o net_io.o coaa1090.obj $(LIBS) $(LDFLAGS)

clean:
	rm -f *.o dump1090 view1090 faup1090 ppup1090

install-doc:
	$(MAKE) -C doc install

install-html:
	install -d $(SHAREDIR)
	cp -R public_html $(SHAREDIR)

install-dump1090: dump1090
	install -d $(BINDIR)
	install -t $(BINDIR) dump1090

install-view1090: view1090
	install -d $(BINDIR)
	install -t $(BINDIR) view1090

install-faup1090: faup1090
	install -d $(BINDIR)
	install -t $(BINDIR) faup1090

install-rcd:
	install -d $(ETCDIR)/init.d
	install -t $(ETCDIR)/init.d fadump1090.sh

install-autostart: install-rcd
	update-rc.d fadump1090.sh defaults

install: install-dump1090 install-view1090 install-faup1090 install-doc install-html
