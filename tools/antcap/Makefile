# make check: prove the SDR tooling against the firmware's own C, then run the
# decoder's synthetic self-test and an antgen -> file -> antdecode round trip.
# Needs: cc, python3 + numpy. No hardware, no HackRF.
ANT      := ../../components/ant
PY       ?= python3
TMP      ?= /tmp/antcap-check
CFLAGS   := -std=c99 -Wall -Wextra -Werror -I$(ANT)/include
CSRC     := crosscheck.c $(ANT)/src/ant_phy_shockburst.c $(ANT)/src/ant_sb_link.c $(ANT)/src/antplus_profiles.c

.PHONY: check crosscheck selftest roundtrip clean

check: crosscheck selftest roundtrip
	@echo "ANTCAP CHECK: PASS"

$(TMP):
	@mkdir -p $(TMP)

crosscheck: | $(TMP)
	@$(CC) $(CFLAGS) -o $(TMP)/crosscheck $(CSRC)
	@$(TMP)/crosscheck > $(TMP)/vectors.c.txt
	@$(PY) antframe.py > $(TMP)/vectors.py.txt
	@diff -u $(TMP)/vectors.c.txt $(TMP)/vectors.py.txt && echo "crosscheck: python == C for $$(wc -l < $(TMP)/vectors.c.txt) vectors"

selftest:
	@$(PY) antdecode.py --selftest

roundtrip: | $(TMP)
	@$(PY) antgen.py -o $(TMP)/hrm.cs8 --seconds 2.5 --devnum 0x3042 --hr 65 --offset 30000 > /dev/null
	@$(PY) antdecode.py $(TMP)/hrm.cs8 --json $(TMP)/hrm.json > $(TMP)/decode.txt
	@$(PY) -c "import json,sys; f=json.load(open('$(TMP)/hrm.json')); \
	    assert len(f)==11, len(f); \
	    assert all(x['body'][:12]=='a6c530427801' for x in f), 'address'; \
	    assert all(x['body'][12:14]=='0a' for x in f), 'ctrl'; \
	    assert all(abs(x['offset_hz']-30000)<5000 for x in f), 'offset'; \
	    import statistics; dt=[b['t']-a['t'] for a,b in zip(f,f[1:])]; \
	    assert all(abs(d-8070/32768)<3e-6 for d in dt), dt; \
	    print('roundtrip: 11/11 frames, address/ctrl/offset/period ok (int8 HackRF file)')"

clean:
	rm -rf $(TMP)
