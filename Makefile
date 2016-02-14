### thesis

all: thesis

thesis:
	./scripts/compile.sh ledanszilard

hunspell:
	hunspell -d hu_HU -i utf-8 -t -p paper/hunspell/words paper/src/ledanszilard.tex

release: gepard thesis
	mkdir -p release
	cp build/ledanszilard.pdf release/
	cp -R code/gepard.git release/

install:
	./scripts/update-deps.sh

clean:
	rm -rf build/*

distclean:
	rm -rf release
	rm -rf build

### gepard

gepard-fetch-code:
	./scripts/gepard-fetch.sh

gepard: gepard-fetch-code
	cd code/gepard.git && make

gepard-clean:
	cd code/gepard.git && make distclean

gepard-rm:
	rm -rf code
