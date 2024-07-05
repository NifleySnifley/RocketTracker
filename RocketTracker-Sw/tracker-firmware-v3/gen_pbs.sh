mkdir -p ./components/fmgr_c/compiled
./components/fmgr_c/generator/nanopb_generator -I ../protocol -D ./components/fmgr_c/compiled ../protocol/protocol.proto
