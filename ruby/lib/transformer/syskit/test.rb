require 'syskit/test/self'
require 'transformer/syskit'

module Transformer
    module SyskitPlugin
        module SelfTest
            include Syskit::Test::Self

            def setup
                super
                Orocos.export_types = true
                Syskit.conf.transformer_enabled = true
            end
        end
        Minitest::Test.include SelfTest
    end
end


