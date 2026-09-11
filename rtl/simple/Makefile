# Simple adapter (rtl/simple) — Icarus Verilog
# Workspace path has spaces: do not mix with the repo-root Verilator Makefile.
# Never compile rtl/adapter_*.v together with this tree.
#
#   make TOP=tb_adapter_simple_unit run
#   make regress
#   make TOP=tb_adapter_simple_qch_to run   # adds -DADP_QCH_TO, DUT=adapter_qch only
#
# Pass string: ALL TESTS PASSED

IVERILOG ?= iverilog
VVP      ?= vvp
TOP      ?= tb_adapter_simple_unit

INCS = -I. -I..
RTL  = adapter_qch.v adapter_atomic.v adapter_err_rsp.v adapter_ip_fail.v \
       adapter_mst_axi_core.v adapter_mst_axi4.v adapter_mst_axi5.v adapter_mst_apb.v \
       adapter_slv_axi_core.v adapter_slv_axi4.v adapter_slv_axi5.v adapter_slv_apb.v
MODELS = sim/adapter_slv_model.v sim/axi_slave_mem.v

ifeq ($(TOP),tb_adapter_simple_qch_to)
DEFS    = -DADP_QCH_TO
RTL_USE = adapter_qch.v
else
DEFS    =
RTL_USE = $(RTL)
endif

VVP_OUT = sim/$(TOP).vvp

.PHONY: all build run regress clean

all: run

build:
	$(IVERILOG) -g2005 $(DEFS) $(INCS) -s $(TOP) -o $(VVP_OUT) \
		$(RTL_USE) $(MODELS) sim/$(TOP).v

run: build
	$(VVP) $(VVP_OUT)

regress:
	$(MAKE) TOP=tb_adapter_simple_unit run
	$(MAKE) TOP=tb_adapter_simple_mst_axi run
	$(MAKE) TOP=tb_adapter_simple_slv_axi run
	$(MAKE) TOP=tb_adapter_simple_apb run
	$(MAKE) TOP=tb_adapter_simple_loopback run
	$(MAKE) TOP=tb_adapter_simple_qch_to run

clean:
	rm -f sim/*.vvp sim/*.vcd
