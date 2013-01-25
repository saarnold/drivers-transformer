module Transformer
    # Implementation of an algorithm that propagates the frame information along
    # the dataflow network, and makes sure that frame selections are consistent.
    class FramePropagation < Syskit::NetworkGeneration::DataFlowComputation
        class FrameAnnotation
            attr_reader :task
            attr_reader :frame_name
            attr_reader :selected_frame

            def initialize(task, frame_name, selected_frame)
                @task, @frame_name, @selected_frame = task, frame_name, selected_frame
            end

            def empty?; !@selected_frame end
            def merge(ann)
                if !ann.kind_of?(FrameAnnotation)
                    raise ArgumentError, "cannot merge a frame annotation with a transform annotation. You are probably connecting two ports, one declared as a transform input or output and one only associated with a frame"
                end

                if ann.selected_frame != selected_frame
                    raise FrameSelectionConflict.new(task, frame_name, selected_frame, ann.selected_frame),
                        "invalid network: frame #{frame_name} in #{task} would need to select both #{ann.selected_frame} and #{selected_frame}"
                end
                false
            end

            def to_s
                "#<FrameAnnotation: #{task} #{frame_name}=#{selected_frame}>"
            end
        end

        # Assignment of a transformation during frame propagation
        class TransformAnnotation
            # The task on which we act
            attr_reader :task
            # The task's frame name for the source of the transformation
            attr_reader :from_frame
            # The selected source frame
            attr_reader :from
            # The task's frame name for the target of the transformation
            attr_reader :to_frame
            # The selected target frame
            attr_reader :to

            def initialize(task, from_frame, from, to_frame, to)
                @task = task
                @from_frame = from_frame
                @from = from
                @to_frame = to_frame
                @to   = to
            end

            # Needed by DataFlowComputation
            #
            # Returns true if neither +from+ nor +to+ are set
            def empty?; !@from && !@to end
            # Merge the information of two TransformAnnotation objects.
            #
            # This succeeds only if the two annotations point to the same
            # frames, or if one has nil and the other does not
            def merge(ann)
                if !ann.kind_of?(TransformAnnotation)
                    raise ArgumentError, "cannot merge a frame annotation with a transform annotation. You are probably connecting two ports, one declared as a transform input or output and one only associated with a frame"
                end

                changed = (@from != ann.from) || (@to != ann.to)
                @from ||= ann.from
                @to   ||= ann.to
                if ann.from && ann.from != from
                    raise FrameSelectionConflict.new(task, from_frame, from, ann.from), "incompatible selection: #{ann.from} != #{@from}"
                end
                if ann.to && ann.to != to
                    raise FrameSelectionConflict.new(task, to_frame, to, ann.to), "incompatible selection: #{ann.to} != #{@to}"
                end
                changed
            end

            def pretty_print(pp)
                pp.text "#{from} => #{to}"
            end

            def to_s # :nodoc:
                "#<TransformAnnotation: #{task} #{from_frame}=#{from} => #{to_frame}=#{to}>"
            end
        end

        def self.compute_frames(plan)
            algorithm = FramePropagation.new
            tasks = plan.find_local_tasks(Syskit::TaskContext).to_value_set
            algorithm.propagate(tasks)
        end

        def required_information(tasks)
            result = Hash.new
            tasks.each do |t|
                next if !(tr = t.model.transformer)

                task_info = [nil]
                tr.each_annotated_port do |port, frame|
                    task_info << port
                end
                result[t] = task_info
            end
            result
        end

        Trigger = Syskit::NetworkGeneration::DataFlowComputation::Trigger

        def triggering_port_connections(task)
            return if !(tr = task.model.transformer)

            interesting_ports = Set.new

            result = Hash.new
            connections = Set.new

            tr.each_annotated_port do |port, frame|
                interesting_ports << port.name
            end
            tr.each_transform_port do |port, transform|
                interesting_ports << port.name
            end

            interesting_ports.each do |port_name|
                task.each_concrete_input_connection(port_name) do |source_task, source_port, _|
                    connections << [source_task, source_port]
                end
                task.each_concrete_output_connection(port_name) do |_, sink_port, sink_task, _|
                    connections << [sink_task, sink_port]
                end

                if !connections.empty?
                    result[port_name] = Trigger.new(connections, Trigger::USE_PARTIAL)
                    connections = Set.new
                end
            end
            result
        end

        def initial_information(task)
            tr = task.model.transformer

            # Add frame information from the devices if there is some
            # This does not require the presence of a transformer spec
            if task.kind_of?(Syskit::Device)
                task.each_master_device do |dev|
                    selected_frame = dev.frame
                    selected_transform = dev.frame_transform

                    # Two cases:
                    #  - the device is using the transformer (has frame
                    #    definitions). Assign the selected frame
                    #  - the device is NOT using the transformer. Therefore,
                    #    we must only add the relevant port info
                    task.find_all_driver_services_for(dev) do |srv|
                        srv.each_task_output_port(true) do |out_port|
                            # Do not associate the ports that output transformations
                            if selected_transform && Transformer.transform_port?(out_port)
                                from, to = selected_transform.from, selected_transform.to
                                if tr && (transform = tr.find_transform_of_port(out_port))
                                    if from
                                        FramePropagation.debug { "selecting #{from} on #{task} for #{transform.from} from the source frame of device #{dev.name}" }
                                        task.select_frame(transform.from, from)
                                    end
                                    if to
                                        FramePropagation.debug { "selecting #{to} on #{task} for #{transform.to} from the target frame of device #{dev.name}" }
                                        task.select_frame(transform.to, to)
                                    end
                                else
                                    add_port_info(task, out_port.name,
                                        TransformAnnotation.new(task, nil, from, nil, to))
                                    done_port_info(task, out_port.name)
                                end
                            elsif selected_frame
                                if tr && (frame_name = tr.find_frame_of_port(out_port))
                                    FramePropagation.debug { "selecting #{selected_frame} on #{task} for #{frame_name} from the frame of device #{dev.name}" }
                                    task.select_frame(frame_name, selected_frame)
                                else
                                    add_port_info(task, out_port.name,
                                        FrameAnnotation.new(task, frame_name, selected_frame))
                                    done_port_info(task, out_port.name)
                                end
                            end
                        end
                    end
                end
            end

            # Now look for transformer-specific information
            return if !tr

            # Now add information for all ports for which we know the frame
            # already
            tr.each_annotated_port do |port, frame_name|
                if selected_frame = task.selected_frames[frame_name]
                    add_port_info(task, port.name, FrameAnnotation.new(task, frame_name, selected_frame))
                    done_port_info(task, port.name)
                end
            end
            tr.each_transform_output do |port, transform|
                from = task.selected_frames[transform.from]
                to   = task.selected_frames[transform.to]
                add_port_info(task, port.name, TransformAnnotation.new(task, transform.from, from, transform.to, to))
                if from && to
                    done_port_info(task, port.name)
                end
            end
            tr.each_transform_input do |port, transform|
                from = task.selected_frames[transform.from]
                to   = task.selected_frames[transform.to]
                add_port_info(task, port.name, TransformAnnotation.new(task, transform.from, from, transform.to, to))
                if from && to
                    done_port_info(task, port.name)
                end
            end

            Transformer.debug do
                Transformer.debug "initially selected frames for #{task}"
                available_frames = task.model.transformer.available_frames.dup
                task.selected_frames.each do |frame_name, selected_frame|
                    Transformer.debug "  selected #{frame_name} for #{selected_frame}"
                    available_frames.delete(frame_name)
                end
                Transformer.debug "  #{available_frames.size} frames left to pick: #{available_frames.to_a.sort.join(", ")}"
                break
            end
        end

        class PortAssociationMismatch < RuntimeError
            # The problematic endpoint, as a [task, port_name] pair
            attr_reader :endpoint
            # The other side of the problematic connection(s) 
            attr_reader :connections
            # The association type expected by +endpoint+. Can either be 'frame'
            # for an association between a port and a frame, and 'transform' for
            # an association between a port and a transformation.
            attr_reader :association_type

            def initialize(task, port_name, type)
                @endpoint = [task, port_name]
                @association_type = type

                @connections = []
                task.each_concrete_input_connection(port_name) do |source_task, source_port_name, _|
                    @connections << [source_task, source_port_name]
                end
                task.each_concrete_output_connection(port_name) do |_, sink_port_name, sink_task, _|
                    @connections << [sink_task, sink_port_name]
                end
            end

            def pretty_print(pp)
                pp.text "#{endpoint[0]}.#{endpoint[1]} was expecting an association with a #{association_type}, but one or more connections mismatch"
                pp.nest(2) do
                    pp.breakable
                    pp.seplist(connections) do |conn|
                        pp.text "#{conn[0]}.#{conn[1]}"
                    end
                end
            end
        end

        # Computes the set of ports and selected frames that can give an insight
        # as to the error represented by +e+
        def refine_invalid_frame_selection(e)
            related_ports = []
            tr = e.task.model.transformer
            tr.each_annotated_port do |port, frame_name|
                next if !has_information_for_port?(e.task, port.name)
                if frame_name == e.frame
                    related_ports << [port.name, :selected_frame]
                end
            end
            tr.each_transform_port do |port, transform|
                next if !has_information_for_port?(e.task, port.name)
                if transform.from == e.frame
                    related_ports << [port.name, :from]
                end
                if transform.to == e.frame
                    related_ports << [port.name, :to]
                end
            end

            related_ports.each do |port_name, accessor|
                info = port_info(e.task, port_name)
                selected_frame = info.send(accessor)

                e.task.each_concrete_input_connection(port_name) do |source_task, source_port, _|
                    e.related_ports << [source_task, source_port, selected_frame]
                end
                e.task.each_concrete_output_connection(port_name) do |source_port, sink_port, sink_task, _|
                    e.related_ports << [sink_task, sink_port, selected_frame]
                end
            end
        end

        def propagate_task(task)
            return if !(tr = task.model.transformer)

            # First, save the port annotations into the select_frames hash on
            # the task.
            tr.each_annotated_port do |port, frame|
                if has_information_for_port?(task, port.name)
                    info = port_info(task, port.name)
                    if !info.respond_to?(:selected_frame)
                        raise PortAssociationMismatch.new(task, port.name, 'frame')
                    end

                    begin
                        debug { "selecting #{info.selected_frame} on #{task} for #{frame} through port #{port.name}" }
                        task.select_frame(frame, info.selected_frame)
                    rescue InvalidFrameSelection => e
                        refine_invalid_frame_selection(e)
                        raise e, e.message, e.backtrace
                    end
                end
            end
            tr.each_transform_port do |port, transform|
                next if !has_information_for_port?(task, port.name)
                info = port_info(task, port.name)

                begin
                    if info.from
                        debug { "selecting #{info.from} on #{task} for #{transform.from} through the source frame of port #{port.name}" }
                        task.select_frame(transform.from, info.from)
                    end
                    if info.to
                        debug { "selecting #{info.to} on #{task} for #{transform.to} through the source frame of port #{port.name}" }
                        task.select_frame(transform.to, info.to)
                    end
                rescue InvalidFrameSelection => e
                    refine_invalid_frame_selection(e)
                    raise e, e.message, e.backtrace
                end
            end

            # Then propagate newly found information to ports that are
            # associated with the frames
            has_all = true
            tr.each_annotated_port do |port, frame_name|
                next if has_final_information_for_port?(task, port.name)

                if selected_frame = task.selected_frames[frame_name]
                    if !has_information_for_port?(task, port.name)
                        add_port_info(task, port.name, FrameAnnotation.new(task, frame_name, selected_frame))
                        done_port_info(task, port.name)
                    end
                else
                    has_all = false
                end
            end
            tr.each_transform_port do |port, transform|
                next if has_final_information_for_port?(task, port.name)
                from = task.selected_frames[transform.from]
                to   = task.selected_frames[transform.to]
                add_port_info(task, port.name, TransformAnnotation.new(task, transform.from, from, transform.to, to))
                if from && to
                    done_port_info(task, port.name)
                else
                    has_all = false
                end
            end
            return has_all
        end

        def self.initialize_selected_frames(task, current_selection)
            # Do selection for the frames that can't be configured anyways
            if task.model.respond_to?(:transformer) && (tr = task.model.transformer)
                tr.each_statically_mapped_frame do |frame_name|
                    debug { "selected frame #{frame_name} on #{task} for #{frame_name}: static frame" }
                    task.select_frame(frame_name, frame_name)
                end
            end

            if task.requirements
                new_selection = task.requirements.frame_mappings
            else
                new_selection = Hash.new
            end

            # If the task is associated to a device, check the frame
            # declarations on the device declaration, and assign them. The
            # assignation is done through the port (i.e. device->port and
            # port->frame), so it requires frame-to-port or transform-to-port
            # associations
            if task.respond_to?(:each_master_device) && (tr = task.model.transformer)
                task.each_master_device do |dev|
                    selected_frame = dev.frame
                    selected_transform = dev.frame_transform

                    # Two cases:
                    #  - the device is using the transformer (has frame
                    #    definitions). Assign the selected frame
                    #  - the device is NOT using the transformer. Therefore,
                    #    we must only add the relevant port info
                    #
                    # This part covers the part where we have to store the frame
                    # selection (first part). The second part is covered in
                    # #initial_information
                    task.find_all_driver_services_for(dev) do |srv|
                        srv.each_task_output_port(true) do |out_port|
                            # Do not associate the ports that output transformations
                            if selected_transform && Transformer.transform_port?(out_port)
                                from, to = selected_transform.from, selected_transform.to
                                if transform = tr.find_transform_of_port(out_port)
                                    if from
                                        new_selection[transform.from] = from
                                    end
                                    if to
                                        new_selection[transform.to] = to
                                    end
                                end
                            elsif selected_frame
                                if frame_name = tr.find_frame_of_port(out_port)
                                    new_selection[frame_name] = selected_frame
                                end
                            end
                        end
                    end
                end
            end

            if new_selection.empty?
                debug { "selecting frames #{current_selection} propagated from its parents" }
                task.select_frames(current_selection)
            else
                debug { "adding frame selection from #{task}: #{new_selection}" }
                task.select_frames(new_selection)
            end
        end

        def self.initialize_transform_producers(task, current_selection)
            if task.requirements
                new_selection = task.requirements.transform_producers
            else
                new_selection = Hash.new
            end

            task.transform_producers =
                if new_selection.empty?
                    current_selection.dup
                else
                    new_selection.dup
                end
        end
    end
end

