
all: thesis

thesis:
	cd paper/ && ./scripts/compile ledanszilard

hunspell:
	hunspell -d hu_HU -i utf-8 -t -p paper/hunspell/words paper/src/ledanszilard.tex

release: thesis
	mkdir -p release
	cp paper/build/ledanszilard.pdf release/

clean:
	rm paper/ledanszilard.pdf
	rm -rf paper/build/*

distclean:
	rm -rf release
	rm paper/ledanszilard.pdf
	rm -rf paper/build
