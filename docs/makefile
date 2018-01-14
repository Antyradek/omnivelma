FILES= main.tex macros.tex title.tex abstract.tex about.tex tools.tex robot.tex model.tex components.tex tests.tex bibliography.tex

all:main.pdf

main.pdf: $(FILES)
	pdflatex -halt-on-error -output-format pdf main.tex
