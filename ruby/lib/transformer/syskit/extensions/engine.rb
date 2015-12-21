module Transformer
    # Module used to add some functionality to Syskit::NetworkGeneration::Engine
    module EngineExtension
        # During network validation, checks that all required frames have been
        # configured
        def validate_generated_network(plan, options)
            super

            if Syskit.conf.transformer_enabled?
                plan.find_local_tasks(Syskit::TaskContext).each do |task|
                    next if !(tr = task.model.transformer)

                    tr.each_needed_transformation do |transform|
                        if !task.selected_frames[transform.from]
                            raise MissingFrame, "could not find a frame assignment for #{transform.from} in #{task}"
                        end
                        if !task.selected_frames[transform.to]
                            raise MissingFrame, "could not find a frame assignment for #{transform.to} in #{task}"
                        end
                    end
                end
            end
        end
    end
end

