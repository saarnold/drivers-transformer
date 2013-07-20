module Transformer
    # Exception raised when a frame is being selected with #selected_frame, but
    # the selection is invalid
    #
    # The reason for the invalidity is (for now) stored only in the message
    class InvalidFrameSelection < RuntimeError
        # The task for which the selection was done
        attr_reader :task
        # The task's frame name
        attr_reader :frame
        # List of [task, port] pairs that give information about why we are
        # updating the frame
        attr_reader :related_ports

        def initialize(task, frame)
            @task, @frame = task, frame
            @related_ports = Array.new
        end

        def pretty_print_related_ports(pp)
            pp.seplist(related_ports) do |src|
                task, port_name, info = *src
                pp.text "#{src[0]}.#{src[1]}: #{info}"
            end
        end
    end

    # Exception raised when two different frames are being selected for the same
    # task/frame_name pair
    class FrameSelectionConflict < InvalidFrameSelection
        # The currently selected frame
        attr_reader :current_frame
        # The newly selected frame
        attr_reader :new_frame

        def initialize(task, frame, current, new)
            super(task, frame)
            @current_frame = current
            @new_frame = new
        end

        def pretty_print(pp)
            pp.text "conflicting frames selected for #{task}.#{frame} (#{current_frame} != #{new_frame}): #{message}"
            if !related_ports.empty?
                pp.breakable
                pp.text "related ports:"
                pp.nest(2) do
                    pp.breakable
                    pretty_print_related_ports(pp)
                end
            end
        end
    end

    # Exception thrown in #merge if the merged task has a different frame
    # selection than the merging task
    class FrameSelectionConflictDuringMerge < FrameSelectionConflict
        # The task that was being merged
        attr_reader :merged_task

        def initialize(merging_task, merged_task, frame, current, new)
            super(merging_task, frame, current, new)
            @merged_task = merged_task
        end

        def pretty_print(pp)
            pp.text "while doing #{task}.merge(#{merged_task})"
            pp.breakable
            super
        end
    end

    # Exception raised when #select_frame is called on a static frame with a
    # different name than the frame's static name
    class StaticFrameChangeError < InvalidFrameSelection
        # The name of the frame that was being assigned to a static frame
        attr_reader :new_frame
        def initialize(task, frame, new)
            super(task, frame)
            @new_frame = new
        end
        def pretty_print(pp)
            pp.text "cannot change frame #{task}.#{frame} to #{new_frame}, as the component does not support it"
            if !related_ports.empty?
                pp.text "related ports:"
                pp.nest(2) do
                    pp.breakable
                    pretty_print_related_ports(pp)
                end
            end
        end
    end

    # Exception raised during network generation if the system cannot find a
    # production chain for a transformation
    class InvalidChain < RuntimeError
        # The task for which the transformation was being produced
        attr_reader :task
        # The task-local name for the source frame
        attr_reader :task_from
        # The task-local name for the target frame
        attr_reader :task_to
        # The global name for the source frame
        attr_reader :from
        # The global name for the target frame
        attr_reader :to
        # The exception explaining the error
        attr_reader :reason

        def initialize(task, task_from, from, task_to, to, reason)
            @task, @task_from, @from, @task_to, @to, @reason =
                task, task_from, from, task_to, to, reason
        end

        def pretty_print(pp)
            pp.text "cannot find a production chain for #{from} => #{to}"
            pp.breakable
            pp.text "  in #{task}"
            pp.breakable
            pp.text "  (task-local: #{task_from} => #{task_to})"
            pp.breakable
            reason.pretty_print(pp)
        end
    end

    # Exception raised during network generation if a declared producer cannot
    # provide the required transformation
    class TransformationPortNotFound < RuntimeError
        attr_reader :task
        attr_reader :from
        attr_reader :to

        def initialize(task, from, to)
            @task, @from, @to = task, from, to
        end

        def pretty_print(pp)
            pp.text "cannot find a port providing the transformation #{from} => #{to} on"
            pp.breakable
            task.pretty_print(pp)
        end
    end
    # Exception raised during network generation if multiple ports can provide
    # a required transformation
    class TransformationPortAmbiguity < RuntimeError
        attr_reader :task
        attr_reader :from
        attr_reader :to
        attr_reader :candidates

        def initialize(task, from, to, candidates)
            @task, @from, @to, @candidates = task, from, to, candidates
        end

        def pretty_print(pp)
            pp.text "multiple candidate ports to provide the transformation #{from} => #{to} on"
            pp.nest(2) do
                pp.breakable
                task.pretty_print(pp)
            end
            pp.breakable
            pp.text "Candidates:"
            pp.nest(2) do
                pp.breakable
                pp.seplist(candidates) do |c|
                    c.pretty_print(pp)
                end
            end
        end
    end

    # Exception raised when a needed frame is not assigned
    class MissingFrame < RuntimeError; end

    # Exception raised when a producer requires itself to function
    class RecursiveProducer < RuntimeError; end

end

