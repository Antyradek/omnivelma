INKSCAPE=inkscape
PDFS=gramofon.pdf

all: $(PDFS)

%.pdf: %.svg
	$(INKSCAPE) -z -A $@ $<
