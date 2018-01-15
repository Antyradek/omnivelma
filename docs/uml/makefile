INKSCAPE=inkscape
PDFS=gramofon.pdf encoders.pdf final.pdf wewucho.pdf

all: $(PDFS)

%.pdf: %.svg
	$(INKSCAPE) -z -A $@ $<
