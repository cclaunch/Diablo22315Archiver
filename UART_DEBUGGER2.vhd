--------------------------------------------------------------------------------
--  Copyright (c) 2017-2020 Eclektek LLC. All rights reserved.
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
--------------------------------------------------------------------------------
--  History :
--
--  Date      By    Comments
--  --------  ----  ------------------------------------------------------------
--  07-24-20  W.C.  Created
--  01-13-25  CVC   Modified for fixed baud rate 115,200 and clock 100MHz
--
-------------------------------------------------------------------------------
--    Project       : Any
--    PCB           : Any
--    Entity Name   : UART_DEBUGGER2
--    File Name     : UART_DEBUGGER2.vhd
--    Design Parent : UART_DEBUGGER.vhd
--
-------------------------------------------------------------------------------
--  Dependencies:
--
--  UART_DEBUGGER2.vhd uses the following components:
--  Component
--    YASUTX.vhd
--  UART_DEBUGGER2.vhd uses the following packages:
--  Package        none
--
-------------------------------------------------------------------------------
--  Description:
--
--    UART_DEBUGGER2 is a tool that converts std_logic_vectors to an ASCII
--    string. The string is appended with CRLF, serialized and sent to the
--    txd_out pin in a form compatible with RS-232 UART protocol.
--
--    UART_DEBUGGER2 is an easy way to view or log the state of signal in
--    FPGA hardware when specific conditions are met. Using an external
--    cable or board that converts compatible digital signals to a USB UART--
--    allows the tool to be used on any board with a spare IO pin on a connector
--    that also supplies a ground.
--
--  Interface:
--    CLK_RATE        A generic input specifying the frequency of clk in Hz
--    BAUD_RATE       A generic input specifying the tranasmitter baud rate
--    DATA_CHARS      A generic input specifying the size of data_in in 4-bit nibbles
--    clk             The clock whose frequency must agree with CLK_RATE
--    areset          An asynchronous reset input
--    data_write_stb  A one clk period wide strobe that starts sending data when asserted high
--    ascii_in        8-bit ascii character to transmit over the UART
--    ascii_write_stb A one clk period wide strobe that sends one ascii character when asserted high
--    data_in         8-bit data input to be transmitted over the UART
--    busy            A flag output that when asserted high indicates that YASUTX
--                    is not free to accept new data_write_stb assertions
--    txd_out         The UART Tx output
--
-------------------------------------------------------------------------------
--  Limitations
--
--    - data_in width must be even multiples of 4
--
-------------------------------------------------------------------------------
--  Deficiencies:
--
-------------------------------------------------------------------------------
--  ToDo:
--
-------------------------------------------------------------------------------
--  NOTES:
-------------------------------------------------------------------------------



Library ieee;
  use ieee.std_logic_1164.all;
  use ieee.std_logic_arith.all;
  use ieee.std_logic_unsigned.all;
  use ieee.math_real.all;

entity UART_DEBUGGER2 is
  generic(
    CLK_RATE        : natural := 100000000;
    BAUD_RATE       : natural := 115200;
    DATA_CHARS      : natural := 8);   -- 32-bit register width = 8*DATA_CHARS);
  port(
        -- GLOBAL SIGNALS
        clk                 : in    std_logic; -- global clock
        areset              : in    std_logic; -- global reset

        -- interface
        data_write_stb      : in    std_logic;
        data_in             : in    std_logic_vector(4*DATA_CHARS-1 downto 0);
        ascii_in            : in    std_logic_vector(7 downto 0);
        ascii_write_stb     : in    std_logic;
        busy                : out   std_logic;

        -- UART signals
        txd_out             : out   std_logic
  );
end entity;

architecture rtl of UART_DEBUGGER2 is

-------------------------------------------------------------------------------
--  Component Declarations
-------------------------------------------------------------------------------


component YASUTX is
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
end component;


-------------------------------------------------------------------------------
--  Type Declarations
-------------------------------------------------------------------------------

type sm_states is (s_idle, s_rd_ascii, s_wr_ascii, s_waita, s_rdchar, s_wr_char, s_shift, s_wait, s_rdcr, s_wr_cr, s_waitcr, s_wr_lf, s_end);
-- Quartus2 Synthesis Options
attribute syn_encoding : string;
attribute syn_encoding of sm_states : type is "safe, one-hot";



-------------------------------------------------------------------------------
--  Constant Declarations
-------------------------------------------------------------------------------

  constant  ASCII_0          : std_logic_vector(7 downto 0) := X"30"; -- ascii '0'
  constant  ASCII_XU         : std_logic_vector(7 downto 0) := X"58"; -- ascii 'X'
  constant  ASCII_xL         : std_logic_vector(7 downto 0) := X"78"; -- ascii 'x'
  constant  ASCII_SPACE      : std_logic_vector(7 downto 0) := X"20"; -- ascii ' '
  constant  ASCII_CR         : std_logic_vector(7 downto 0) := X"0D"; -- ascii control carriage return
  constant  ASCII_LF         : std_logic_vector(7 downto 0) := X"0A"; -- ascii control Line Feed
  constant  ASCII_WU         : std_logic_vector(7 downto 0) := X"57"; -- ascii 'W'
  constant  ASCII_wL         : std_logic_vector(7 downto 0) := X"77"; -- ascii 'w'
  constant  ASCII_RU         : std_logic_vector(7 downto 0) := X"52"; -- ascii 'R'
  constant  ASCII_rL         : std_logic_vector(7 downto 0) := X"72"; -- ascii 'r'
--  constant  UART_BAUD_RATE_DIVISOR          : natural := integer(round(real(CLK_RATE)/real(BAUD_RATE)));        -- baud period in clocks
--  constant  UART_BAUD_RATE_DIVISOR_WIDTH    : natural := integer(ceil(log2(real(UART_BAUD_RATE_DIVISOR))));    -- 1+ CEILING(LOG2(1786))
  constant  UART_BAUD_RATE_DIVISOR          : natural := 868;                                               -- 115200 baud period in clocks
  constant  UART_BAUD_RATE_DIVISOR_WIDTH    : natural := 11;                                                -- hold 868 with sign 0
  constant  DEFAULT_ADDRESS                 : natural := 0;
  constant  DEFAULT_REG_DATA                : natural := 0;
  constant  TX_LINE_CHAR_WIDTH              : natural := DATA_CHARS + 2;  -- plus '0x', ' 0x', '\n'

-------------------------------------------------------------------------------
--  Signal Declarations
-------------------------------------------------------------------------------

signal  current_state, next_state        : sm_states;

signal  data_r            : std_logic_vector(4*DATA_CHARS-1 downto 0);
signal  arst_n            : std_logic;
signal  tx_char           : std_logic_vector(7 downto 0);
signal  tx_char1          : std_logic_vector(7 downto 0);
signal  uart_tx_full      : std_logic;
signal  uart_wren         : std_logic;
signal  char_count        : natural range 0 to DATA_CHARS;
signal  valid_hex         : std_logic;
signal  is_delimit        : std_logic;
signal  tx_error          : std_logic;

-------------------------------------------------------------------------------
--  Procedure Declarations
-------------------------------------------------------------------------------


begin


-------------------------------------------------------------------------------
--  Component Instantiations
-------------------------------------------------------------------------------

  UART0 : YASUTX
  generic map(
    CLK_RATE        => CLK_RATE,
    BAUD_RATE       => BAUD_RATE)
  port MAP(
     clk             => clk,
     areset          => areset,
     data_write_stb  => uart_wren,
     data_in         => tx_char1,
     busy            => uart_tx_full,
     tx_error        => tx_error,
     txd_out         => txd_out
  );


-------------------------------------------------------------------------------
--  Concurrent Assignments
-------------------------------------------------------------------------------

  arst_n        <= not areset;

  uart_wren     <=  '1' when current_state = s_wr_char else
                    '1' when current_state = s_wr_cr else
                    '1' when current_state = s_wr_lf else
                    '1' when current_state = s_wr_ascii else
                    '0';


  tx_char       <= X"30" when data_r((data_r'left) downto (data_r'left)-3) = X"0" else
                   X"31" when data_r((data_r'left) downto (data_r'left)-3) = X"1" else
                   X"32" when data_r((data_r'left) downto (data_r'left)-3) = X"2" else
                   X"33" when data_r((data_r'left) downto (data_r'left)-3) = X"3" else
                   X"34" when data_r((data_r'left) downto (data_r'left)-3) = X"4" else
                   X"35" when data_r((data_r'left) downto (data_r'left)-3) = X"5" else
                   X"36" when data_r((data_r'left) downto (data_r'left)-3) = X"6" else
                   X"37" when data_r((data_r'left) downto (data_r'left)-3) = X"7" else
                   X"38" when data_r((data_r'left) downto (data_r'left)-3) = X"8" else
                   X"39" when data_r((data_r'left) downto (data_r'left)-3) = X"9" else
                   X"41" when data_r((data_r'left) downto (data_r'left)-3) = X"A" else
                   X"42" when data_r((data_r'left) downto (data_r'left)-3) = X"B" else
                   X"43" when data_r((data_r'left) downto (data_r'left)-3) = X"C" else
                   X"44" when data_r((data_r'left) downto (data_r'left)-3) = X"D" else
                   X"45" when data_r((data_r'left) downto (data_r'left)-3) = X"E" else
                   X"46" when data_r((data_r'left) downto (data_r'left)-3) = X"F" else
                   X"20";

  tx_char1      <= ASCII_CR when current_state = s_wr_cr else
                   ASCII_CR when current_state = s_rdcr else
                   ASCII_LF when current_state = s_waitcr else
                   ASCII_LF when current_state = s_wr_lf else
                   ascii_in when current_state = s_rd_ascii else
                   ascii_in when current_state = s_wr_ascii else
                   tx_char;


-------------------------------------------------------------------------------
--  Process Declarations
-------------------------------------------------------------------------------


  DATAIN_LATCH_PROC : process(clk, areset)
  begin
    if (areset = '1') then
      data_r   <= (others => '0');
    elsif rising_edge(clk) then
      if data_write_stb = '1' then
        data_r   <= data_in;
      elsif (current_state = s_shift) then
        data_r   <= data_r((data_r'left)-4 downto 0) & X"0";
      else
        data_r   <= data_r;
      end if;
    end if;
  end process DATAIN_LATCH_PROC;

  CHAR_COUNT_PROC : process(clk, areset)
  begin
    if areset = '1' then
      char_count   <= 0;
    elsif rising_edge(clk) then
      if current_state = s_end then
        char_count   <= 0;
      elsif data_write_stb = '1' then
        char_count   <= DATA_CHARS;
      elsif (current_state = s_shift) then
        char_count   <= char_count - 1;
      else
        char_count <= char_count;
      end if;
    end if;
  end process CHAR_COUNT_PROC;


  -- the 2 process state machine
  UPDATE_STATE : process(clk, areset)
  begin
    if (areset = '1') then
      current_state   <= s_idle;
    elsif rising_edge(clk) then
      current_state   <= next_state;
    end if;
  end process UPDATE_STATE;


--type sm_states is (s_idle, s_rd_ascii, s_wr_ascii, s_waita, s_rdchar, s_wr_char, s_shift, s_wait, s_rdcr, s_wr_cr, s_waitcr, s_wr_lf, s_end);

  CONTROL_SM : process (current_state, data_write_stb, uart_tx_full, char_count, ascii_write_stb)
  begin
    case current_state is
      when s_idle =>
        if (uart_tx_full = '0' and data_write_stb = '1') then
          next_state  <= s_rdchar;
        elsif (uart_tx_full = '0' and ascii_write_stb = '1') then
          next_state  <= s_rd_ascii;
        else
          next_state  <= s_idle;
        end if;

      when s_rd_ascii =>
        next_state  <= s_wr_ascii;

      when s_wr_ascii =>
        next_state  <= s_waita;

      when s_waita =>
        if uart_tx_full = '0' then
          next_state  <= s_idle;
        else
          next_state  <= s_waita;
        end if;

      when s_rdchar =>
        next_state  <= s_wr_char;

      when s_wr_char =>
        next_state  <= s_shift;

      when s_shift =>
        next_state  <= s_wait;

      when s_wait =>
        if uart_tx_full = '0' and char_count > 0 then
          next_state  <= s_rdchar;
        elsif  uart_tx_full = '0' and char_count = 0 then
          next_state  <= s_rdcr;
        else
          next_state  <= s_wait;
        end if;

      when s_rdcr =>
        next_state  <= s_wr_cr;

      when s_wr_cr =>
        if uart_tx_full = '0' then
          next_state  <= s_waitcr;
        else
          next_state  <= s_wr_cr;
        end if;

      when s_waitcr =>
        if uart_tx_full = '0' then
          next_state  <= s_wr_lf;
        else
          next_state  <= s_waitcr;
        end if;

      when s_wr_lf =>
        next_state  <= s_end;

      when s_end =>
        next_state  <= s_idle;

      when others =>
        next_state  <= s_idle;

    end case;
  end process CONTROL_SM;



-------------------------------------------------------------------------------
--  Other Output Assignments
-------------------------------------------------------------------------------

  busy           <= '0' when current_state = s_idle and uart_tx_full = '0' else '1';

end rtl;
