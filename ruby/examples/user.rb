$LOAD_PATH.unshift File.expand_path(File.join('..', 'lib'), File.dirname(__FILE__))
load 'transformer.rb'

class TestChecker
    def test(producer)
	puts "blub"
	raise
    end
end

tr = Transformer.new() do |i|
	puts "producer is "
	pp i
end

tr.load_configuration('example.rb')

tr_chain = tr.get_transformation_chain(:laser, :head)
pp tr_chain
