//==========================================================================================================
// Diablo 2315 Archiver
// File Name: archiver.v
//
// Written for 2315 on Diablo by Carl Claunch
//
//==========================================================================================================

// This routine will loop, calling bus_read_drive to read a sector at a time
// It will look at the Sector address and the sector pulses
//
// Set up Cyl, Head, Sector and reset Word address to 0 for each sector 
// call bus_read_drive which will pulse grabword and present the data in readdata
// we advance the word address after the write has finished 
// we interlock waiting for readdone from bus_read_drive
// sector read saved the syncerror and ECCerror status in word 322 of DRAM
//
// step through the four sectors on a surface, as above.
// change the head to the other surface and step through four more sectors
//
// request a seek of one track, which updates the Cylinder value and informs us when done
// for each cylinder 0 to 202, do the two surfaces by four sectors each as above
// signal completion when we have completed the cartridge
//
// This is started by a pushbutton on the FPGA board

module archiver (

// Inputs
    input wire clock,                     // system clock
    input wire reset,                     // system reset
    input wire extract_cartridge,         // ask us to fetch the contents of the 2315 pack
    input wire [1:0] Sector_Address,      // next sector value from sector_index module
    input wire readdone,                  // when sector reading is complete
    input wire READY_SEEK_L,              // completion signal from seek module

// Outputs
    output reg requestread,               // trigger the bus_read_drive module to read a sector
    output reg Head_Select,               // which surface
    output reg strobe_move,               // ask for one track advance
    output reg alldone                    // indicates the archiving is complete

);

//============================ Internal Connections ==================================

//============================ Logic =================================================

// for each new track, set head to 0 first
// at chosen sector, set requestread and wait bus_read_drive to complete
// each word is grabbed by dram_module as it is read
// when readdone is turned on, we wait for Sector to become 01 then raise requestread again
// this reads second sector. In the same way, wait for Sector to be 10 and then 11, 
// each time raising requestread to get it read.
// after fourth sector, set head to 1 and repeat the inner logic for the four sectors
//
// after both heads are done, call seek_to_cylinder to move one track forward then redo
// give up after we do cylinder 202
//
// we have inner that does sectors 0 to 3
// with inner-inner that does one sector
// middle does heads 0 and 1
// outer does cylinders 0 to 202

`define AR0 3'd0
`define AR1 3'd1
`define AR2 3'd2
`define AR3 3'd3
`define AR4 3'd4
`define AR5 3'd5
`define AR6 3'd6
`define AR7 3'd7
// 0 is idle
// 1 wait for sector to match target
// 2 call bus_read_drive to archive sector
// 3 advances sector target, 01 to 02 to 03
// 4 swaps Head
// 5 advances cylinder target
// 6 seek to new cylinder
// 7 is end wait


reg [2:0] archive_state; // archive machine state variable

reg [3:0]  metabutton;         // metastability chain for button 0 input that starts archiving
reg [1:0]  target_sector;      // which sector to read next
reg [1:0]  metaready;          // detect edge
wire        readyout;
reg        current_head;       // are we working on top or bottom head now
reg [1:0]  last_sector;        // used to detect sector changes
reg        new_sector;         // flag when it changes
reg [7:0]  current_cylinder;   // used to track cylinders


//=============================== start of code ======================================//
always @ (posedge clock)
begin : ARCHIVE // block name
  if(reset) begin
    archive_state    <= `AR0;
    metabutton       <= 4'b0000;
    metaready        <= 2'b00;
    target_sector    <= 2'b00;
    current_head     <= 1'b0;
    last_sector      <= Sector_Address; 
    requestread      <= 1'b0;
    current_cylinder <= 8'd0;
    strobe_move      <= 1'b0;
    alldone          <= 1'b0;
    new_sector       <= 1'b0;
    Head_Select      <= 1'b0;
  end
  else begin

     // button has bounce and is not in our clock domain, so deal with metastability
     metabutton[3:0] <= {metabutton[2:0], extract_cartridge};

     // edge detectors
     metaready[1:0]  <= {metaready[0], readyout};

     Head_Select <= current_head;

     new_sector <= last_sector != Sector_Address
                   ? 1'b1
                   : 1'b0;

     case(archive_state)

// sitting around waiting to begin our work when button 0 pushed
    `AR0: begin  

       // next state determination
       archive_state <=  metabutton[3] == 1'b0 && metabutton[2] == 1'b1
                         ? `AR1
                         : `AR0;

       // we always start with sector 0 of each head/track
       target_sector <= 2'b0;

       // we start with upper head (head 0)
       current_head <= 1'b0;

       // housekeeping
       requestread <= 1'b0;
       strobe_move <= 1'b0;
       alldone <= 1'b0;

    end   

// waiting for our target sector
    `AR1: begin  

       // next state determination
       // bail out when past end of cartridge, else wait for 
       // target sector to approach
       archive_state <= current_cylinder == 8'd203
                        ? `AR7
                        : new_sector && Sector_Address == target_sector 
                          ? `AR2
                          : `AR1;

       // housekeeping
       requestread <= 1'b0;
       strobe_move <= 1'b0;
       alldone <= 1'b0;

    end   

// read the sector
    `AR2: begin  

       // next state determination
       // we fired off bus_read_drive and wait for readdone
       // while it grabs 321 words from sector and stores in RAM
       archive_state <= readdone == 1'b1
                     ? `AR3
                     : `AR2;

       // emit request for read
       requestread <= 1'b1;

       // grab the errors if any 
       

       // housekeeping
       strobe_move <= 1'b0;
       alldone <= 1'b0;

    end   

// advance sector within head
    `AR3: begin  
       // next state determination
       // once the bus_read_drive goes to idle
       // we repeat for sectors 01, 10 and 11
       archive_state <=  readdone == 1'b1
                         ? `AR3
                         : target_sector == 2'b11
                           ? `AR4
                           : `AR1;

       // bump the target sector
       target_sector <= readdone == 1'b1
                        ?  target_sector
                        :  target_sector + 1;

       // housekeeping
       requestread <= 1'b0;
       strobe_move <= 1'b0;
       alldone <= 1'b0;

    end   

// switch to other head
    `AR4: begin  

       // next state determination
       // we move from upper track to lower track
       // for another four sectors before advancing
       archive_state <= current_head == 1'b1
                        ?  `AR5
                        :  `AR1;

       // flip the current head
       current_head <= ~current_head;

       // housekeeping
       requestread <= 1'b0;
       strobe_move <= 1'b0;
       alldone <= 1'b0;

    end   

// advance to next track (Cylinder) target
    `AR5: begin  

       // next state determination
       // we advance our cylinder pointer here
       archive_state <= `AR6;

      // bump cylinder we will archive up until 202
      current_cylinder <= current_cylinder + 1;

       // assert strobe_move for one cycle
       strobe_move <= 1'b1;
       
       // housekeeping
       requestread <= 1'b0;
       alldone <= 1'b0;

    end   

// perform seek
    `AR6: begin  

       // next state determination
       // keep looping on all cylinders until past last one
       archive_state <= current_cylinder == 8'd203
                        ? `AR7
                        :  metaready[1] == 1'b1 && metaready[0] == 1'b0  // drive goes ready again
                           ? `AR1
                           : `AR6;

       // drop strobe_move 
       strobe_move <= 1'b0;

       // housekeeping
       requestread <= 1'b0;
       alldone <= 1'b0;

    end   

// wrap up after archiving complete for cartridge
    `AR7: begin  

       // next state determination
       // lock ourselves here for duration
       archive_state <= `AR7;

       // inform we are done with the archive
       alldone <= 1'b1;

       // housekeeping
       requestread <= 1'b0;
       strobe_move <= 1'b0;

    end   

    default: begin
      archive_state <= `AR0;
    end

    endcase

  end
end // End of Block ARCHIVE
   
threshold hysteresisR (
        .clk (clock),
        .reset (reset),
        .sigin (~READY_SEEK_L),
        .sigout (readyout)
   );  

endmodule // archiver
