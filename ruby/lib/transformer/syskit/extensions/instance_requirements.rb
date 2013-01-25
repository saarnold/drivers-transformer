module Transformer
    # Module used to extend the instance specification objects with the ability
    # to map frame names to global frame names
    #
    # A frame mapping applies recursively on all levels of the hierarchy. I.e.
    # it applies on the particular instance and on all its children.
    module InstanceRequirementsExtension
        # The set of frame mappings defined on this specification
        attribute(:frame_mappings) { Hash.new }
        # The set of transformation producers defined on this specification
        attribute(:transform_producers) { Hash.new }

        # Declare frame mappings
        #
        # For instance, doing
        #
        #   use_frames("world" => "odometry")
        #
        # will assign the "odometry" global frame to every task frame called "world".
        def use_frames(frame_mappings)
            self.frame_mappings.merge!(frame_mappings)
            self
        end

        # Declare a specialized transformation producer for this requirements
        # and its children
        def use_transform_producer(from, to, producer)
            self.transform_producers[[from, to]] = producer
            self
        end

        # Updates the frame mappings when merging two instance requirements
        def merge(other_spec)
            super if defined? super

            frame_mappings.merge!(other_spec.frame_mappings) do |frame_name, sel0, sel1|
                if !sel0 then sel1
                elsif !sel1 then sel0
                elsif sel0 != sel1
                    raise ArgumentError, "cannot merge #{self} and #{other_spec}: frame mapping for #{frame_name} differ (resp. #{sel0.inspect} and #{sel1.inspect})"
                else
                    sel0
                end
            end

            transform_producers.merge!(other_spec.transform_producers) do |(from, to), sel0, sel1|
                if sel0 != sel1
                    raise ArgumentError, "cannot merge #{self} and #{other_spec}: producer for #{from} => #{to} differ (resp. #{sel0} and #{sel1})"
                end
                sel0
            end
        end

        # Displays the frame mappings when pretty-print an InstanceRequirements
        # object
        #
        # Unlike "normal" pretty-printing, one must always start with a
        # pp.breakable
        def pretty_print(pp)
            super if defined? super

            pp.breakable
            pp.text "Frame Selection:"
            if !frame_mappings.empty?
                pp.nest(2) do
                    pp.breakable
                    pp.seplist(frame_mappings) do |mapping|
                        pp.text "#{mapping.first} => #{mapping.last}"
                    end
                end
            end
        end
    end
end


