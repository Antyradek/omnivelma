FILES= main.tex macros.tex title.tex abstract.tex about.tex tools.tex robot.tex model.tex implementation.tex tests.tex ending.tex bibliography.tex

all:main.pdf

main.pdf: $(FILES)
	pdflatex -shell-escape -halt-on-error -output-format pdf main.tex
