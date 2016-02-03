### thesis

all: thesis

thesis:
	./scripts/compile.sh ledanszilard

hunspell:
	hunspell -d hu_HU -i utf-8 -t -p paper/hunspell/words paper/src/ledanszilard.tex

release: thesis
	mkdir -p release
	cp build/ledanszilard.pdf release/

clean:
	rm -rf build/*

distclean:
	rm -rf release
	rm -rf build

### gepard

gepard-fetch-code:
	./scripts/gepard-fetch.sh

gepard:
	cd code/gepard.git && make debug

gepard-clean:
	cd code/gepard.git && make release.clean
	cd code/gepard.git && make debug.clean

gepard-rm:
	rm -rf code
