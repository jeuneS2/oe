# Clock in input pin (50 MHz)
create_clock -period 20 [get_ports clk]

# Create generated clocks based on PLLs
derive_pll_clocks -use_tan_name

derive_clock_uncertainty
