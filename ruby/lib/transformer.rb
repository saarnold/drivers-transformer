require 'pp'
require 'utilrb/kernel/load_dsl_file'

class Frame
    attr_accessor :name    
end

class Transformation
   attr_reader :from
   attr_reader :to   

   def initialize(from, to)
       @from = from
       @to = to
   end
   
    def pretty_print(pp)
	pp.text " #{from} to #{to} "
    end

    def inverse
	return Transformation.new(to, from)
    end
    
end


class StaticTransformation < Transformation
end

class DynamicTransformation < Transformation
   attr_reader :producer
   
   def initialize(from, to, producer)
       super(from, to)
       @producer = producer
   end

    def inverse
	return Transformation.new(to, from, producer)
    end

end



class TransformationChain < Transformation
    #This must be an Array of Transformation 
    #links representing the TransformationChain
    attr_reader :links

    def initialize(transformation_node)
	if(!transformation_node.is_a?(TransformationNode))
	    throw("Internal API error, wrong type given")
	end

	@links = Array.new

	to = transformation_node.frame;
	cur_node = transformation_node
	while(cur_node.parent) do
	    @links.insert(0, cur_node.link_to_parent)
	    cur_node = cur_node.parent
	end

	super(cur_node.frame, to)
    end
    
    def pretty_print(pp)
	pp.text "Transformation Chain: #{from} to #{to} "
	pp.text pp.newline
	pp.text "Links: #{pp.newline}[#{pp.newline}"
	links.each do |i|
	    pp i
	end
	pp.text "]#{pp.newline}"
    end

    
end

class TransformationNode
    attr_reader :parent
    attr_reader :frame
    attr_reader :link_to_parent
    
    def initialize(frame, parent, link_to_parent)
       @parent = parent
       @frame = frame
       @link_to_parent = link_to_parent
    end
    
end

class Transformer
    
    attr_accessor :configuration
    attr_accessor :checker
    attr_reader :max_seek_depth
    
    def initialize(&producer_check)
	@max_seek_depth = 50;
	@checker = ConfigurationChecker.new(producer_check)
    end
    
    def load_configuration(config_file)
 	@configuration = ConfigurationParser.new(@checker)
	
	eval_dsl_file(config_file, @configuration, [], false)
# 	@configuration.instance_eval(IO.read(config_file))
    end
        
    def get_matching_transformations(node, transforms)
	ret = Array.new
	
	transforms.each do |i|
	    if(i.to == node.frame && (!node.parent || i.from != node.parent.frame))
		ret.push(i.inverse)
	    end

	    if(i.from == node.frame && (!node.parent || i.to != node.parent.frame))
		ret.push(i)
	    end
	end

	return ret
    end
    
    def get_transformation_chain(from, to)
	if(!configuration)
	    throw("Not initialized yet, did you forget to load the configuration ?")
	end
	
	checker.check_frame(from, configuration.frames)
	checker.check_frame(to, configuration.frames)
	
	all_transforms = Array.new
	all_transforms.concat(configuration.dynamic_transformations)
	all_transforms.concat(configuration.static_transformations)
	
	possible_next_nodes = Array.new
	possible_next_nodes.push(TransformationNode.new(from, nil, nil))
	
	for i in 0..@max_seek_depth
	    next_level = Array.new
	    
	    if(possible_next_nodes.empty?)
		raise "No transformation from '#{from}' to '#{to}' available"
	    end
	    
	    possible_next_nodes.each do |node|
		links_for_node = get_matching_transformations(node, all_transforms)
		
		links_for_node.each do |link|
		    next_level.push(TransformationNode.new(link.to, node, link))
		end
	    end
	    
	    next_level.each do |candidate|
		if(candidate.frame == to)
		   puts ("Found chain")
		   return TransformationChain.new(candidate)
		end
	    end

	    possible_next_nodes = next_level
	end
	throw("Max seek depth reached seeking Transformation from '#{from}' to #{to}")
	
    end
    
end

class ConfigurationChecker
    attr_accessor :producer_check
    
    def initialize(producer_check)
	@producer_check = producer_check
    end
	
    def check_transformation_frames(frames, transformations)
	transformations.each do |i|
	    check_transformation(frames, i)
	end
    end

    def check_transformation(frames, transformation)
	error = ""
	if(!frames.include?(transformation.from))
	    error.concat("Error: Transformation from #{transformation.from} to #{transformation.to} uses unknown frame #{transformation.from} \n")
	end	
	    
	if(!frames.include?(transformation.to))
	    error.concat("Error: Transformation from #{transformation.from} to #{transformation.to} uses unknown frame #{transformation.to}")
	end
	if(error != "")
	    raise(error)
	end
    end
    
    def check_frame(frame, frames = nil)
	if(!frame.is_a? Symbol)
	    raise "Error: Frames must be Symbols, frame '#{frame}' is not a symbol" 
	end
	
	if(frames && !frames.include?(frame))
	    raise ("Error: Frame '#{frame}' is not known")
	end
    end
    
    def check_producer(producer)
	if(@producer_check)
	    @producer_check.call(producer)
	end
    end
end

class ConfigurationParser
    attr_accessor :dynamic_transformations
    attr_accessor :static_transformations
    attr_accessor :frames
    attr_accessor :checker
    
    def initialize(checker)
	@dynamic_transformations = Array.new
	@static_transformations = Array.new
	@frames = Array.new
	@checker = checker
    end
    
    def available_frames(*frames)
	frames.each do |i|
	    checker.check_frame(i)
	end
	@frames.concat(frames)
    end

    def DynamicTransformation(from, to, producer)
	checker.check_producer(producer)
	tr = DynamicTransformation.new(from, to, producer)
	checker.check_transformation(frames, tr)
	dynamic_transformations.push(tr)
	
    end
    
    def StaticTransformation(from, to)
	tr = StaticTransformation.new(from, to)
	checker.check_transformation(frames, tr)
	static_transformations.push(tr)
    end
    
    def pretty_print(pp)
	pp.text "Available Frames: "
	pp.text pp.newline
	frames.each do |i| 
	    pp.text " #{i}"
	    pp.text pp.newline
	end
	
	pp.text pp.newline
	
	pp.text "Static Transformations: "
	pp.text pp.newline
	static_transformations.each do |i|
	    pp i
	end
	pp.text pp.newline
	
	pp.text "Dynamic Transformations: "
	pp.text pp.newline
	dynamic_transformations.each do |i|
	    pp i
	end
        
    end
    
end

