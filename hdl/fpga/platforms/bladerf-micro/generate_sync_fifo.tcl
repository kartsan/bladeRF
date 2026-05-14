# TCL script to generate Altera Synchronous FIFO IP
# Run with: quartus_sh -t generate_sync_fifo.tcl

# Set project directory
set project_dir "y:/Ilkka/ham/bladerf/bladeRF/hdl/fpga/platforms/bladerf-micro"

# Create IP generation script
set ip_name "sync_fifo_net"
set ip_dir "$project_dir/ip/altera/sync_fifo"

# Ensure directory exists
file mkdir $ip_dir

# Generate the FIFO using megafunction wizard (command line equivalent)
# Note: This is a simplified version; full generation may require qsys

# For Quartus, use qsys-generate or direct instantiation
# Since sync_fifo is a standard megafunction, it can be instantiated directly
# But to generate HDL, use:

puts "Generating Synchronous FIFO IP..."
puts "IP Name: $ip_name"
puts "Location: $ip_dir"
puts "Width: 32 bits"
puts "Depth: 16 words"
puts "Show-ahead: OFF"

# In practice, run:
# quartus_ipgenerate --project=bladerf-hosted --ip=$ip_name --output=$ip_dir

# Then add to project
puts "Add the generated files to your Quartus project manually or via script."