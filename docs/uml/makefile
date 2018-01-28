INKSCAPE=inkscape
PDFS=comparison.pdf final.pdf

all: $(PDFS)

%.pdf: %.svg
	$(INKSCAPE) -z -A $@ $<
