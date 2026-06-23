//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 06/14/2026 10:12:06 AM
// Design Name: 
// Module Name: hysteresis
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: Provides hystersis in switching input signal on or off
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module hysteresis(
    input wire clk,
    input wire reset,
    input wire sigin,
    output reg sigout
);

reg [2:0] count = 3'b0;
reg [2:0] onthresh = 3'd5;
reg [2:0] offthresh = 3'd1;

always @ (posedge clk)
begin : HYSTER // block name
reg [3:0]  metain;            // metastability chain for input

  if(reset==1'b1) begin 
    sigout <= 1'b0;
    count = 2'd0;
  end
  else begin
      sigout <= (count > onthresh)
                        ? 1'b1
                        : (count < offthresh)
                            ? 1'b0
                            : sigout; 
      count <= (metain[3] == 1'b1)
                ? (count < 2'd7)
                    ? count + 1
                    : count
                : (count > 2'd0)
                   ? 0
                   : count - 1;
  end


end // End of Block HYSTER
 
endmodule
