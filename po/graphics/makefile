INKSCAPE=inkscape
PDFS=base.pdf

all: $(PDFS)

%.pdf: %.svg
	$(INKSCAPE) -z -A $@ $<
