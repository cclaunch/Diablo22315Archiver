//==========================================================================================================
// Diablo Archiver
// read disk from the BUS
// File Name: bus_disk_read.v
// Functions: 
//   extracts real data from a sector of the Diablo disk drive 
//   and stores it in block RAM for extraction over USB
//   will see the read_clock and read_data signals from the disk drive when
//   we switch them on with read_gate. 
// Modified for Diablo reading 2315 cartridges by Carl Claunch
//
//==========================================================================================================

module bus_disk_read(
    // INPUTS
    input wire clock,               // master clock 12 MHz
    input wire reset,               // active high synchronous reset input
    input wire requestread,         // ask module to read a sector
    input wire clkenbl_sector,      // we got a sector pulse
    input wire RD_DATA_L,           // Read data pulses from drive
    input wire RD_CLK_L,            // Read clock pulses from drive

    // OUTPUTS
    output reg RD_GATE_L,           // Read gate, when active enables read circuitry
    output reg [15:0] readdata,     // 16-bit read data to the block ram controller
    output reg readdone,            // confirmation we are done
    output reg [8:0] Word_Number,   // which word did we read
    output reg grabword             // request write into block ram

);

//============================ Internal Connections ==================================

// state definitions and values for the read state
`define BRST0 3'd0 // 0 - off
`define BRST1 3'd1 // 1 - wait through preamble
`define BRST2 3'd2 // 2 - find first Sync bit
`define BRST3 3'd3 // 3 - grab ECC of 1110 from sync word
`define BRST4 3'd4 // 4 - extract Data words
`define BRST5 3'd5 // 5 - ignore Postamble
`define BRST6 3'd6 // 6 - stopped by sector pulse  
`define BRST7 3'd7 // 7 - write sync or ECC error status as word 322  
reg [2:0] bus_read_state; // read state machine state variable

// define asserted and deasserted state of output signal
`define ASSERT 1'b1
`define DEASSERT 1'b0

// IBM 1130 2315 cartridge has 20 bit words, 321 words with no CRC, 4 logical sectors (8 physical)
// data word is 16 bits, ECC is 1 bits emitted in last four until count mod 4 is 0
//
// read begins when -Read Gate goes low (at the end of a sector pulse
// drive sends 250 us of zeroes (clock pulses with no data pulses) as preamble
// we then send the sync word - a word with value 0000000000000001 plus check bits 1110
// following sync we grab data from DRAM and send it out on the clock and data lines
// 
// for each word we bump a two bit counter for each 1 data bit we grab
// this will be 00 when the count of 1 bits is evenly divisible by four
//
// we have to extract the 16 bit cells for the data then watch the four check bits
// they are emitted as 1 and the counter bumped till it hits 00 then we send 0 bits
//
// the postamble should be all zero bit values until the next sector arrives
// 

reg [7:0]  bus_read_count; // count us in a word or Preamble length
reg [15:0] psreg; // parallel-to-serial register, send data LSB first
reg [11:0] wordcount; // counter to keep track of the number of data words, in 16-bit increments
reg [7:0]  sync_word_count;   // count bits during sync word tail end
reg [1:0]  ECC_count;         // count one bits for ECC generation
reg [1:0]  metaclock;         // edge detector for Read Clock input
wire        clockout;         // from hysteresis threshold detector
reg [1:0]  metadata;          // edge detector for Read Data input
wire        dataout;          // from hysteresis threshold detector
reg        gotone;            // did Read_Data get a bit while clock is low?
reg        bitready;          // result ready in gotone
reg        ECCerror;          // one or more words failed the error detect check
reg        syncerror;         // the sync word was not 00000000000000011110 

//============================ Start of Code =========================================


always @ (posedge clock)
begin : DISKREAD // block name

  if(reset==1'b1) begin
    readdone <= 1'b0;
    grabword <= 1'b0;
    bus_read_state <= `BRST0;
    bus_read_count <= 8'd0;
    metaclock <= 2'b00;
    metadata <= 2'b00;
    wordcount <= 12'd0;
    psreg <= 16'd0;
    gotone <= 1'b0;
    bitready <= 1'b0;
    readdone <= 1'b0;
    sync_word_count <= 8'd0;
    readdata <= 16'd0;
    ECC_count <= 2'd0;
    RD_GATE_L <= `DEASSERT;
    syncerror <= 1'b0;
    ECCerror <= 1'b0;
    Word_Number <= 9'd0;
  end
  else begin
    // track old and new states to find edges
    metaclock[1:0] <= {metaclock[0], clockout};
    metadata[1:0] <= {metadata[0], dataout};

    // bit capture logic
    gotone <= (metaclock[1] == 1'b1 && metaclock[0] == 1'b0)  // falling edge of clock
              ? 1'b0                                          // reset
              : (metaclock[1] == 1'b0 && metaclock[0] == 1'b0 && metadata[1] == 1'b1)
                ? 1'b1                                        // data bit 1 while clock 0
                : gotone;

    // tell when we have a 1 bit detected
    
    bitready <= ((metaclock[1] == 1'b0) && (metaclock[0] == 1'b1) &&
                (wordcount != 0)) // rising edge of clock
                ? 1'b1
                : 1'b0;

    case(bus_read_state)
// transmission is off, waiting for the read gate
    `BRST0: begin     
      // when to move out of idle state
      bus_read_state <= (requestread == 1'b1 && clkenbl_sector == 1'b1)
                        ? `BRST1 
                        : `BRST0;

      // ready to skip 100 bits of Preamble before watching for Sync
      bus_read_count <= (requestread == 1'b1) 
                        ? 14'd100               // full preamble is 195 but 100 bits is long
                        : 14'd0;                // enough for clock separator to stabilize


      // reset shift register and word count
      psreg <= 16'd0;
      wordcount <= 12'd0;

      readdata <= 16'd0;

      // read gate is off
      RD_GATE_L <= `DEASSERT;
      readdone <= 1'b0;
      syncerror <= 1'b0;
      ECCerror <= 1'b0;
      grabword <= 1'b0;
      Word_Number <= 9'd0;
      sync_word_count <= 0;
     end
// ignore Preamble of all 0's
    `BRST1: begin     

      // decrement bit count if either clock or data is high
      bus_read_count <= ((metaclock[1] == 1'b1 && metaclock[0] == 1'b0) 
                        || (metadata[1] == 1'b1 && metadata[0] == 1'b0))
                        ? bus_read_count - 1 
                        : bus_read_count;

      // move to sync word when our preamble count goes to 1
      bus_read_state <= (((bus_read_count == 8'd1) && metaclock[1] == 1'b1 && metaclock[0] == 1'b0) 
                           ? `BRST2 
                           : `BRST1);

      // zero out shift register and word count
      psreg <= 16'd0;
      wordcount <= 12'd0;

      readdata <= 16'd0;

      // get the count of words in a sector - 321 for 2310 disk drive
      wordcount <= 12'd321;

      // read gate is on
      RD_GATE_L <= `ASSERT;
      readdone <= 1'b0;
      syncerror <= 1'b0;
      ECCerror <= 1'b0;
      sync_word_count <= 0;
      grabword <= 1'b0;
      Word_Number <= 9'd0;

     end

// wait for the Sync bit
    `BRST2: begin     

      // set up for usual 20 bit word in data state `BRST4
      bus_read_count <= 8'd20;


      // found the sync word (15 bits of 0, bit of 1)   
      bus_read_state <= (bitready == 1'b1 && gotone == 1'b1)
                           ? `BRST3 
                           : `BRST2;


      // get the count of words in a sector - 321 for 2310 disk drive
      wordcount <= 12'd321;

      readdata <= 16'd0;

      // read gate is on
      RD_GATE_L <= `ASSERT;
      readdone <= 1'b0;
      sync_word_count <= 0;
      grabword <= 1'b0;
      Word_Number <= 9'd0;

     end

// verify ECC on the sync word
    `BRST3: begin     

      // set up for usual 20 bit word in data state `BRST4
      bus_read_count <= 8'd20;

      // build up the bits
      psreg <=  (sync_word_count == 4)
                ?  16'd0
                :  (bitready == 1'b1)
                   ? {gotone, psreg[15:1]}
                   : psreg;

      // count off the three
      sync_word_count <= (bitready == 1'b1) 
                         ? ((sync_word_count == 4) 
                            ? 8'd0 
                            : sync_word_count + 1) 
                         : sync_word_count;

      // found the sync word (15 bits of 0, bit of 1)   
      bus_read_state <= (sync_word_count == 4)
                           ? `BRST4 
                           : `BRST3;

      // correct ECC is 1110 for the sync word
      syncerror <= (sync_word_count == 3)
                   ? (psreg[15:12] == 4'b1110)
                     ? 1'b0
                     : 1'b1
                   : syncerror;

      // get the count of words in a sector - 321 for 2310 disk drive
      wordcount <= 12'd321;

      readdata <= 16'd0;

      // read gate is on
      RD_GATE_L <= `ASSERT;
      readdone <= 1'b0;
      grabword <= 1'b0;
      Word_Number <= 9'd0;

     end

// read data words of the sector
    `BRST4: begin     

      // build up the bits
      psreg <= ((bus_read_count == 8'd1) && metaclock[1] == 1'b1 && metaclock[0] == 1'b0) 
               ?     16'd0
               :     (bitready == 1'b1) && (bus_read_count > 14'd4)
                     ? {gotone, psreg[15:1]}
                     : psreg;

      // count one bits for the ECC in each word
      ECC_count <= (bitready == 1'b1 && gotone == 1'b1)
                   ? (bus_read_count == 14'd1)
                        ? 2'd0
                        : ECC_count + 1 
                   : ECC_count;

      // check ECC must be 'sum 1 bits' modulo 4 == 0
      ECCerror <= ((bus_read_count == 8'd1) && metaclock[1] == 1'b1 && metaclock[0] == 1'b0)
                   ? ECC_count == 2'b00
                     ? ECCerror
                     : 1'b1
                   : ECCerror;
 
      // decrement rising clock and rollover from 0 to 19 if the count is at zero, 
      // otherwise hold at the present count
      bus_read_count <= bitready == 1'b1
                        ? ((bus_read_count == 8'd1) 
                               ? 8'd20
                               : bus_read_count - 1) 
                        : bus_read_count;

      // if done with sector, graceful stop
      bus_read_state <= ((bus_read_count == 8'd1) && (wordcount == 12'd0) && 
                         (metaclock[1] == 1'b1) && (metaclock[0] == 1'b0))
                     ? `BRST7             
                     : `BRST4;


      // decrement word count 
      wordcount <= ((bus_read_count == 8'd1) && (metaclock[1] == 1'b1) && 
                    (metaclock[0] == 1'b0) && (wordcount != 0)) 
                   ? wordcount - 1 
                   : wordcount;

      // emit our word number
      Word_Number <=  ((bus_read_count == 8'd1) && metaclock[1] == 1'b0 && metaclock[0] == 1'b1)
                      ? Word_Number + 1
                      : Word_Number;

      // load the word to be written
      readdata <= ((bus_read_count == 8'd1) && metaclock[1] == 1'b1 && metaclock[0] == 1'b0)
                  ? psreg
                  : readdata;

      // let the archiver know a word is ready
      grabword <= ((bus_read_count == 8'd1) && (metaclock[1] == 1'b1) 
                    && (metaclock[0] == 1'b0) && (wordcount != 0))
                  ? 1'b1
                  : 1'b0;

      // read gate is on
      RD_GATE_L <= `ASSERT;
      readdone <= 1'b0;

     end

// ignore Postamble of all zeroes
    `BRST5: begin     

      // just one bit 
      bus_read_count <= 8'd0;

      // continue ignoring until either
      // read gate goes off or we reach the next sector
      bus_read_state <= (requestread == 1'b0) 
                        ? `BRST0 
                        : (clkenbl_sector == 1'b1)
                          ? `BRST6
                          : `BRST5;
 
      // zero out register and count
      psreg <= 16'd0;
      wordcount <= 12'd0;
      grabword <= 1'b0;

      // read gate is off
      RD_GATE_L <= `DEASSERT;
      readdone <= 1'b1;

     end

// overran sector pulse, must stop giving time for requestread to drop
    `BRST6: begin     


      bus_read_state <= requestread == 1'b0
                        ? `BRST0
                        : `BRST6;
 
      psreg <= 16'd0;
      wordcount <= 12'd0;
      grabword <= 1'b0;
      bus_read_count <= 8'd0;
      readdata <= 16'd0;
      RD_GATE_L <= `DEASSERT;
      readdone <= 1'b0;

     end

// record the sync and ECC errors in word 322
// bit 3 set on to indicate we read this in from disk
    `BRST7: begin     


      bus_read_state <=  `BRST5;

      psreg <= 16'd0;
      wordcount <= 12'd0;
      grabword <=       bitready
                        ? 1'b1
                        : 1'b0;
      bus_read_count <= 8'd0;
      readdata <=       bitready
                        ? {syncerror, ECCerror, 1'b1, 13'b0}
                        : readdata;
      RD_GATE_L <= `DEASSERT;
      readdone <= 1'b0;

     end


    default: begin
      bus_read_state <= `BRST0;
    end

    endcase

  end
end // End of Block DISKREAD

threshold hysteresisC (
        .clk (clock),
        .reset (reset),
        .sigin (~RD_CLK_L),
        .sigout (clockout)
   );  
   
 threshold hysteresisD (
        .clk (clock),
        .reset (reset),
        .sigin (~RD_DATA_L),
        .sigout (dataout)
   );  
     
endmodule // End of Module bus_disk_read
