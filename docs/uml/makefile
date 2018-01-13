INKSCAPE=inkscape
PDFS=gramofon.pdf encoders.pdf

all: $(PDFS)

%.pdf: %.svg
	$(INKSCAPE) -z -A $@ $<
