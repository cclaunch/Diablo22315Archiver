-------------------------------------------------------------------------------
--  Copyright (c) 2017 Eclektek LLC. All rights reserved.
--
--
--  Developed by: William Carter
--
--  Permission is hereby granted, free of charge, to any person obtaining a copy of this software and  
--  associated documentation files (the "Software"), to deal with the Software without restriction,  
--  including without limitation the rights to use, copy, modify, merge, publish, sublicense and/or    
--  distribute copies of the Software, and to permit persons to whom the Software is furnished to do so,  
--  subject to the following conditions:
--
--      Redistributions of source code must retain the above copyright notice, this list of conditions 
--      and the following disclaimers.
--      Redistributions in binary form must reproduce the above copyright notice, this list of conditions 
--      and the following disclaimers in the documentation and/or other materials provided with the distribution.
--      Neither the names of William Carter, Eclektek LLC, nor the names of its 
--      contributors may be used to endorse or promote products derived from this Software without 
--      specific prior written permission from Eclektek LLC.
--      This Software is provided free of charge and Eclektek LLC reserves the right to sell copies of the 
--      Software in it original form. Anyone sub-licensing this Software will provide it free of charge.
--
--  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT 
--  LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. 
--  IN NO EVENT SHALL THE CONTRIBUTORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, 
--  WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE 
--  SOFTWARE OR THE USE OR OTHER DEALINGS WITH THE SOFTWARE.
-------------------------------------------------------------------------------
--  History :                                                                       
--                                                                                                                  
--  Date      By    Comments                                         
--  --------  ----  ------------------------------------------------------------      
--  06-17-17  W.C.  Created                                     
--  06/23/17  W.C.  Addressed bad commentary                                    
--  01/13/25  CVC   Modified to hard code 115,200 baud rate
-------------------------------------------------------------------------------
--    Project       : Any                                                          
--    PCB           : Any 
--    Entity Name   : YASUTX
--    File Name     : YASUTX.vhd
--    Design Parent : none                                                          
--                                                                             
-------------------------------------------------------------------------------
--  History :                                                                       
--                                                                                                                  
--  Date      By    Comments                                         
--  --------  ----  ------------------------------------------------------------      
--  06-20-17  W.C.  Created
--  06-21-17  W.C.  Fixed the busy flag and FIFO                                    
--
-------------------------------------------------------------------------------
--  Dependencies:
--
--  YASUTX.vhd is uses the following components:
--  Component      
--    Uart.vhd
--  YASUTX.vhd is uses the following packages:
--  Package        none
--
-------------------------------------------------------------------------------
--
--  Description:
--
--    Yet Another Simple Uart Transmitter
--
--    Talking is easy. Listening takes some effort.
--
--    YASUTX is a trivial UART transmitter with 8-bit data, 1 start, 1 stop bit
--    no parity and no flow control.
--
--  Interface:
--    CLK_RATE        A generic input specifying the frequency of clk in Hz
--    BAUD_RATE       A generic input specifying the tranasmitter baud rate
--    clk             The clock whose frequency must agree with CLK_RATE  
--    areset          An asynchronous reset input
--    data_write_stb  A one clk period wide strobe that starts sending data when
--                    asserted high
--    data_in         8-bit data input to be transmitted over the UART                                                      
--    busy            A flag output that when asserted high indicates that YASUTX
--                    is not free to accept new data_write_stb assertions
--    tx_error        An error flag output indicating that a data_write_stb assertion
--                    has been ignored because the YASUTX FIFO is full. This flag
--                    is asserted high when busy = '1' and data_write_stb = '1' and
--                    stays high until the state machine returns to an idle state  
--    txd_out         The UART Tx output
--
-------------------------------------------------------------------------------
--  Limitations:
--
-------------------------------------------------------------------------------
--  Deficiencies:
--
-------------------------------------------------------------------------------
--  ToDo:
--
-------------------------------------------------------------------------------
--  NOTES:
-- 
-------------------------------------------------------------------------------



Library ieee;
  use ieee.std_logic_1164.all;
  use ieee.std_logic_arith.all;
  use ieee.std_logic_unsigned.all;
  use ieee.math_real.all;
  
entity YASUTX is
  generic(
    CLK_RATE        : natural := 100000000;
    BAUD_RATE       : natural := 115200);   
  port(
        -- GLOBAL SIGNALS
        clk                 : in    std_logic; -- global clock
        areset              : in    std_logic; -- global reset
        
        -- interface
        data_write_stb      : in    std_logic;
        data_in             : in    std_logic_vector(7 downto 0);
        busy                : out   std_logic;
        tx_error            : out   std_logic;
        
        -- UART signals
        txd_out             : out   std_logic
  );
end entity;

architecture rtl of YASUTX is

-------------------------------------------------------------------------------
--  Component Declarations
-------------------------------------------------------------------------------


-------------------------------------------------------------------------------
--  Type Declarations
-------------------------------------------------------------------------------

type sm_states is (s_idle, s_load, s_shift, s_bit);
-- Quartus2 Synthesis Options
attribute syn_encoding : string;
attribute syn_encoding of sm_states : type is "safe, one-hot";



-------------------------------------------------------------------------------
--  Constant Declarations
-------------------------------------------------------------------------------

-- constant  UART_BAUD_RATE_PERIOD          : natural := integer(round(real(CLK_RATE)/real(BAUD_RATE)));    -- baud period in clocks
-- constant  UART_BAUD_RATE_PERIOD_WIDTH    : natural := integer(ceil(log2(real(UART_BAUD_RATE_PERIOD))));  -- 1+ CEILING(LOG2(869)) 
-- constant  TRANSMIT_BITS                  : std_logic_vector(3 downto 0) := X"A"; -- start & data & stop
--        generated baud rate is 115,207 which is off by only .0064%
constant  UART_BAUD_RATE_PERIOD          : std_logic_vector(10 downto 0) := "01101100100";     --clock period 8.680 us
constant  UART_BAUD_RATE_PERIOD_WIDTH    : integer := 10;                                      -- holds up to 1023 with sign of 0
constant  TRANSMIT_BITS                  : std_logic_vector(3 downto 0) := X"A";               -- start & data & no parity & stop
constant  START_BIT                      : std_logic := '0';
constant  STOP_BIT                       : std_logic := '1';

-------------------------------------------------------------------------------
--  Signal Declarations
-------------------------------------------------------------------------------

signal  current_state        : sm_states;

signal  data_r            : std_logic_vector(7 downto 0); 
signal  shift_r           : std_logic_vector(9 downto 0); 
signal  uart_tx_full      : std_logic;
signal  tx_err            : std_logic;
-- signal  baud_period_cnt   : std_logic_vector(UART_BAUD_RATE_PERIOD_WIDTH-1 downto 0);
signal  baud_period_cnt   : std_logic_vector(10 downto 0);                                      -- good enough for 115200
signal  bit_cnt           : std_logic_vector(3 downto 0);
signal  sending_flg       : std_logic;

-------------------------------------------------------------------------------
--  Procedure Declarations
-------------------------------------------------------------------------------
  
  
begin


-------------------------------------------------------------------------------
--  Component Instantiations
-------------------------------------------------------------------------------

-------------------------------------------------------------------------------
--  Concurrent Assignments
-------------------------------------------------------------------------------
                    
-------------------------------------------------------------------------------
--  Process Declarations
-------------------------------------------------------------------------------

  -- an one deep FIFO
  -- only save incoming tx data if the FIFO is empty
  process(clk, areset)
  begin
    if (areset = '1') then
      data_r        <= (others => '0');
    elsif rising_edge(clk) then
      if data_write_stb = '1' and uart_tx_full = '0' then
        data_r    <= data_in;
      end if;  
    end if;
  end process;

  -- keep track of when the "FIFO" has been written but not read
  process(clk, areset)
  begin
    if (areset = '1') then
      uart_tx_full  <= '0';
    elsif rising_edge(clk) then
      if data_write_stb = '1' then
        uart_tx_full   <= '1';
      elsif  data_write_stb = '0' and current_state = s_load then
        uart_tx_full   <= '0';
      else
        uart_tx_full   <= uart_tx_full;
      end if;
    end if;
  end process;

  -- flag an error if data is lost
  process(clk, areset)
  begin
    if (areset = '1') then
      tx_err  <= '0';
    elsif rising_edge(clk) then
      if data_write_stb = '1' and uart_tx_full = '1' then
        tx_err   <= '1';
      elsif current_state = s_idle then
        tx_err   <= '0';
      else
        tx_err   <= tx_err;
      end if;
    end if;
  end process;
  
  -- The transmitter
-- type sm_states is (s_idle, s_load, s_shift, s_bit);
  TX_SM : process (clk, areset)
  begin
    if (areset = '1') then
      shift_r         <= (others => '1');
      current_state   <= s_idle;
      baud_period_cnt <= (others => '0');
      bit_cnt         <= (others => '0');
    elsif rising_edge(clk) then
      case current_state is
        when s_idle =>
          if data_write_stb = '1' then  
            current_state   <= s_load;
          else
            current_state  <= s_idle;
          end if;
                 
        when s_load =>
          shift_r     <= STOP_BIT & data_r & START_BIT;  -- data is sent D0, D1... D7
          baud_period_cnt <= (others => '0');
          bit_cnt         <= (others => '0');
          current_state  <= s_bit;
        
        -- make sure that the appropriate bit period has passed bewteen bits           
        when s_bit =>
          baud_period_cnt   <= baud_period_cnt + 1;
          if baud_period_cnt > 868 then
            current_state  <= s_shift;
          else
            current_state  <= s_bit;
          end if;  
                 
        when s_shift =>
          baud_period_cnt   <= (others => '0');
          bit_cnt   <= bit_cnt + 1;
          shift_r   <= '1' & shift_r(9 downto 1);
          if bit_cnt < TRANSMIT_BITS then
            current_state  <= s_bit;
          elsif uart_tx_full = '1' then
            current_state  <= s_load;
          else
            current_state  <= s_idle;
          end if;  
          
        when others =>
          current_state  <= s_idle;
        
      end case;
    end if;      
  end process TX_SM; 
 


-------------------------------------------------------------------------------
--  Other Output Assignments
-------------------------------------------------------------------------------

  busy          <= uart_tx_full;
  tx_error      <= tx_err;
  txd_out       <= shift_r(0);

end rtl;
