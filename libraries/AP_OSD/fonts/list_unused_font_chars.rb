#!/usr/bin/env ruby

used_symbols = File.open('../AP_OSD_Backend.h').each_line.inject([]) do |acc, line|
    mdata = line.match /SYM_\w+\s*=\s*(0x[0-9a-zA-Z]{2})\s*;/
    mdata.nil? ? acc : acc.push(Integer(mdata[1]))
end

decimal_symbols = (0xC0..0xC9).to_a + (0xD0..0xD9).to_a
battery_symbols = (0x90..0x96).to_a
arrow_symbols = (0x60..0x6F).to_a
ahi_symbols = (0x80..0x88).to_a
sidebar_symbols = (0x8A..0x8F).to_a
ascii_chars = [ 0x20, 0x21, 0x25, *(0x28..0x2F) ]
unused_symbols = (0x0..0xFE).to_a - (?A..?Z).map(&:ord) - (?0..?9).map(&:ord) - decimal_symbols - battery_symbols - arrow_symbols - ahi_symbols - sidebar_symbols - ascii_chars - used_symbols
unused_symbols_hex = unused_symbols.map { "0x%02X" % _1 }

puts "Unused symbols: #{unused_symbols_hex.join(', ')}"
