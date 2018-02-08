INKSCAPE=inkscape
PDFS=base.pdf mecanum_dirs_vect.pdf

all: $(PDFS)

%.pdf: %.svg
	$(INKSCAPE) -z -A $@ $<
