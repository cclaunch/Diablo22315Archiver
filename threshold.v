`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 06/14/2026 11:19:13 AM
// Design Name: 
// Module Name: threshold
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module threshold(
    input wire clk,
    input wire reset,
    input wire sigin,
    output reg sigout
    );
    
reg [3:0] count = 4'b0;
reg [2:0] onthresh = 3'd5;
reg [2:0] offthresh = 3'd1;

always @ (posedge clk)
begin : HYSTER // block name
reg [3:0]  metain;            // metastability chain for input

  if(reset==1'b1) begin 
    sigout <= 1'b0;
    count = 4'd0;
    metain <= 4'd0;
  end
  else begin
  
      // handle metastability of input signal
      metain[3:0] <= {metain[2:0],sigin};
      
      // output flips with hysteresis - Schmitt trigger
      sigout <= (count > onthresh)
                        ? 1'b1
                        : (count < offthresh)
                            ? 1'b0
                            : sigout; 
                            
      // count up or down based on input signal
      count <= (metain[3] == 1'b1)
                ? (count < 4'd7)
                    ? count + 1
                    : count
                : (count > 4'd0)
                       ?  count - 1
                        : count;
  end


end // End of Block HYSTER
 
endmodule
