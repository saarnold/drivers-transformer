$LOAD_PATH.unshift File.expand_path(File.join('..', 'lib'), File.dirname(__FILE__))
require 'transformer'
require 'pp'

tr = Transformer::Transformer.new do |i|
    puts "producer is "
    pp i
end

tr.load_configuration('example.rb')

tr_chain = tr.transformation_chain(:laser, :head)
pp tr_chain
