//==========================================================================================================
// Diablo Archiver
// sector and index pulse timing generators
// File Name: sector_and_index.v
// Functions: 
//   The 2310 has eight physical sector pulses but controller divides by 2 to see four sectors
//
// Modified for 2310 by Carl Claunch
//
//==========================================================================================================

module sector_and_index(
    input wire clock,                          // master clock 12 MHz
    input wire reset,                          // active high synchronous reset input
    input wire SECTOR_MARK_L,                  // sector pulse from the 2310
    input wire INDEX_MARK_L,                   // index pulse from the 2310
    output reg clkenbl_sector,                 // enable for disk read clock
    output reg [1:0] Sector_Address            //counter that specifies which sector is present "under the heads"
);

//============================ Internal Connections ==================================

reg [1:0] meta_bus_sector;
wire       sectorout;
reg [1:0] meta_bus_index;
wire       indexout;
reg [0:0] eat_pulses;
reg [3:0] PSector_Address;  

//============================ Start of Code ========================================= 

always @ (posedge clock)
begin : SECTORCOUNTERS // block name

  if(reset == 1'b1) begin
    PSector_Address <= 3'd0;
    Sector_Address <= 2'd0;
    clkenbl_sector <= 1'b0;
    meta_bus_sector <= 2'd0;
    meta_bus_index <= 2'd0;
    eat_pulses <= 1'd0;
  end
  else begin

    // detect edges
    meta_bus_sector[1:0] <= {meta_bus_sector[0], sectorout};
    meta_bus_index[1:0] <= {meta_bus_index[0], indexout};

    eat_pulses <= (meta_bus_index[1] == 1'b1 && meta_bus_index[0] == 1'b0)
                  ? 1'b0
                  : (meta_bus_sector[1] == 1'b1 && meta_bus_sector[0] == 1'b0)
                      ? eat_pulses + 1
                      : eat_pulses;

    // indicate falling edge of Sector
    clkenbl_sector <= (meta_bus_sector[1] == 1'b1 && meta_bus_sector[0] == 1'b0 && eat_pulses[0] == 1'b0)
                      ? 1'b1
                      : 1'b0;

    // count physical sectors
    PSector_Address <= (meta_bus_index[1] == 1'b1 && meta_bus_index[0] == 1'b0)
                       ? 3'd0
                       : (meta_bus_sector[1] == 1'b1 && meta_bus_sector[0] == 1'b0)
                         ? (PSector_Address == 3'd7) 
                             ? 3'd0 
                             : PSector_Address + 1
                         : PSector_Address;

    // emit sector address as 1130 system knows it
    Sector_Address <= {1'b0 , PSector_Address[2:1]};

  end

end // End of Block COUNTERS

threshold hysterI (
        .clk (clock),
        .reset (reset),
        .sigin (~INDEX_MARK_L),
        .sigout (indexout)
);  

threshold hysterS (
        .clk (clock),
        .reset (reset),
        .sigin (~SECTOR_MARK_L),
        .sigout (sectorout)
);  

endmodule // End of Module sector_and_index
