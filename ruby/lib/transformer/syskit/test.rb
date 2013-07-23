require 'syskit/test'
require 'transformer/syskit'

module Transformer
    module SyskitPlugin
        module SelfTest
            include Syskit::SelfTest

            def setup
                super
                Orocos.export_types = true
                Syskit.conf.transformer_enabled = true
            end
        end
    end
end

