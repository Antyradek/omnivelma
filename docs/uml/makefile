INKSCAPE=inkscape
PDFS=gramofon.pdf encoders.pdf final.pdf

all: $(PDFS)

%.pdf: %.svg
	$(INKSCAPE) -z -A $@ $<
