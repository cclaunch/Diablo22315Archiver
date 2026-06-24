//==========================================================================================================
// Diablo Archiver
// seek to cylinder logic
// File Name: seek_to_cylinder.v
// Functions: 
//   Receive the movement strobe and move forward one track.
//   Save the new cylinder address in Cylinder_Address.
//
//   We provide the track address then raise STROBE
//   ADDR ACK is returned within 22.5 to 42.5 us
//   must drop STROBE within 5 uS of getting ADDR ACK
//   
//   READY_SEEK_L goes high 2.5 us after STROBE and 
//   remains high until the seek is complete
//
// Modified for 2310 by Carl Claunch
//
//==========================================================================================================

module seek_to_cylinder(
    input wire clock,                  // master clock 12 MHz
    input wire reset,                  // active high synchronous reset input
    input wire strobe_move,            // request from our logic to advance 1 track 
    input wire ADDR_ACK,               // Diablo accepted seek request
    input wire READY_SEEK_L,           // Diablo able to perform a seek

    output reg STROBE_L,               // strobe to enable movement of the heads when low
    output reg [7:0] TRACK,            // command to Diablo for target cylinder
    output reg [7:0] Cylinder_Address  // internal register to store the valid cylinder address
);

//============================ Internal Connections ==================================

// Diablo disk drive is commanded to move to a target cylinder number
// we will only step forward one track at a time

    reg [9:0] countdown;        // hold STROBE pulse for 3 us after ADDR ACK
    reg [2:0] seek_state;       // state machine variable
    
// state definitions and values for seeking
`define S0 3'd0 // 0 - idle
`define S1 3'd1 // 1 - set up the target track
`define S2 3'd2 // 2 - request STROBE, wait for ADDR ACK
`define S3 3'd3 // 3 - prepare to drop STROBE
`define S4 3'd4 // 4 - wait for READY_SEEK_L to restore

// defines for logic level of asseert and deassert
`define ASSERT 1'b1
`define DEASSERT 1'b0
`define NOTRACK 8'b00000000
    
//============================ Start of Code =========================================

always @ (posedge clock)
begin

    if(reset == 1'b1) begin
        Cylinder_Address <= 8'd0;
        STROBE_L <= `DEASSERT;
        countdown <= 10'd0;
        TRACK <= `NOTRACK;
        seek_state <= `S0;
    end
    else begin

    case(seek_state)
// waiting for a request to seek
    `S0: begin   
    
         // no request to move the arm
         STROBE_L <= `ASSERT;
            
         // when strobe activated, bump cylinder location
         Cylinder_Address <= (strobe_move == 1'b1) 
                          // move 1 track forward unless already at 202
                          ?  ((Cylinder_Address + 1) < 203)
                             ? Cylinder_Address + 1
                             : 202
                         : Cylinder_Address;
                         

         seek_state <= (strobe_move == 1'b1)
              ?  `S1
              :  `S0;
                    
         end  
         
// set up the target track and let the signals settle
   `S1: begin
           
        // send cylinder we want to drive
        TRACK <= Cylinder_Address;
                
        // move forward next cycle
        seek_state <= `S2;
        end
        
// strobe requested but need acknowledgement from drive
   `S2: begin
   
        // we request a seek
        STROBE_L <= `ASSERT;
                
        // set up counter to wait 3 us
        countdown <= 10'd300;
        
        // move forward when ADDR_ACK goes low from drive
        seek_state <= (ADDR_ACK == 1'b0)
              ?  `S3
              :  `S2;
        end
        
// keep STROBE_L active for 3 us after ADDR_ACK
   `S3: begin
   
        // continue to hold STROBE_L active
        STROBE_L <= `ASSERT;
        
        // count it off
        countdown <= (countdown == 10'd0)
               ?    0
               :    countdown - 1;
   
        // move forward when countdown gets to 0
        seek_state <= (countdown == 10'd0)
              ?  `S4
              :  `S3;
              
        // drop TRACK when the count expires
        TRACK <= (countdown == 10'd0)
              ?   `NOTRACK
              :   TRACK;
              
        end
        
// wait until drive finished and restores READY_SEEK_L
   `S4: begin
   
        // we deassert STROBE and wait for the move to finish
        STROBE_L <= `DEASSERT;
   
        // go back to idle when the drive completes the seek
        seek_state <= (READY_SEEK_L == 1'b0)
               ?   `S0
               :   `S4;
        end
        
    default: begin
      seek_state <= `S0;
      STROBE_L <= `DEASSERT;
    end

    endcase
 end 
end

endmodule // End of Module sector_and_index
