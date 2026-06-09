source ../../../ip/nuand/nuand.do
compile_nuand ../../../ip/nuand bladerf-micro

vcom -work nuand -2008 ../vhdl/tb/nios_system.vhd

vcom -work nuand -2008 ../../../platforms/common/bladerf/vhdl/fx3_gpif_p.vhd
vcom -work nuand -2008 ../../../platforms/common/bladerf/vhdl/fx3_gpif.vhd

vcom -work nuand -2008 ../vhdl/bladerf_p.vhd
vcom -work nuand -2008 ../vhdl/bladerf.vhd
vcom -work nuand -2008 ../vhdl/rx.vhd
vcom -work nuand -2008 ../vhdl/tx.vhd
vcom -work nuand -2008 ../vhdl/eem/eem_sync_fifo.vhd
vcom -work nuand -2008 ../vhdl/eem/eem_rx_consumer.vhd
vcom -work nuand -2008 ../vhdl/eem/eem_tx_framer.vhd
vcom -work nuand -2008 ../vhdl/eem/cv_chip_id_reader.vhd
vcom -work nuand -2008 ../vhdl/eem/chip_id_mac.vhd
vcom -work nuand -2008 ../vhdl/eem/eth_rx_demux.vhd
vcom -work nuand -2008 ../vhdl/eem/arp_responder.vhd
vcom -work nuand -2008 ../vhdl/eem/ip_rx_handler.vhd
vcom -work nuand -2008 ../vhdl/eem/icmp_responder.vhd
vcom -work nuand -2008 ../vhdl/eem/udp_rx_handler.vhd
vcom -work nuand -2008 ../vhdl/eem/dhcp_client.vhd
vcom -work nuand -2008 ../vhdl/eem/udp_tx_pkg.vhd
vcom -work nuand -2008 ../vhdl/eem/ip_hdr_checksum.vhd
vcom -work nuand -2008 ../vhdl/eem/hpsdr_discovery_responder.vhd
vcom -work nuand -2008 ../vhdl/eem/hpsdr_hp_status_sender.vhd
vcom -work nuand -2008 ../vhdl/eem/hpsdr_hp_cmd_handler.vhd
vcom -work nuand -2008 ../vhdl/eem/hpsdr_ddc_spec_handler.vhd
vcom -work nuand -2008 ../vhdl/eem/hpsdr_cmd_mux.vhd
vcom -work nuand -2008 ../vhdl/eem/hpsdr_dc_blocker.vhd
vcom -work nuand -2008 ../vhdl/eem/hpsdr_ddc_iq_sender.vhd
vcom -work nuand -2008 ../vhdl/eem/tx_arbiter.vhd
vcom -work nuand -2008 ../vhdl/bladerf-hosted.vhd

vcom -work nuand -2008 ../vhdl/tb/fx3_pll.vhd
vcom -work nuand -2008 ../vhdl/tb/system_pll.vhd
vcom -work nuand -2008 ../vhdl/tb/bladerf_tb.vhd

compile_nuand_tb ../../../ip/nuand bladerf-micro

