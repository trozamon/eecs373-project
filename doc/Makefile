BAD_EXTS := aux log out

all : $(patsubst %.tex, %.pdf, $(wildcard *.tex))
	rm -f $(patsubst %, *.%, $(BAD_EXTS))

%.pdf : %.tex
	pdflatex $<
	pdflatex $<
