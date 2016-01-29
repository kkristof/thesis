### thesis

all: thesis

thesis:
	./scripts/compile ledanszilard

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
# TODO: Improve code fetching
	mkdir -p code
	cd code \
	&& git clone git@github.com:szledan/gepard.git gepard.git \
	&& cd gepard.git \
	&& git checkout origin/path_rendering

gepard:
	cd code/gepard.git && make debug

gepard-clean:
	cd code/gepard.git && make release.clean
	cd code/gepard.git && make debug.clean

gepard-rm:
	rm -rf code
