INKSCAPE=inkscape
PDFS=base_dims.pdf friction.pdf base_vects.pdf mecanum_dirs.pdf vectors.pdf

all: $(PDFS)

%.pdf: %.svg
	$(INKSCAPE) -A $@ $<
