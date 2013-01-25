module Transformer
    module ConfigurationExtension
        def transformation_manager
            @transformation_manager ||= Transformer::TransformationManager.new
        end

        def transformer
            transformation_manager.conf
        end
    end
end

