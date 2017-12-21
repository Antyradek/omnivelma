INKSCAPE=inkscape
PDFS=base_dims.pdf friction.pdf base_vects.pdf mecanum_dirs.pdf mecanum_dirs_vect.pdf vectors.pdf weiti_logo.pdf comm.pdf gramofon.pdf

all: $(PDFS)

%.pdf: %.svg
	$(INKSCAPE) -A $@ $<
