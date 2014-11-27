#ifndef TRANSFORMER_TRANSFORMER_STATUS_HPP
#define TRANSFORMER_TRANSFORMER_STATUS_HPP

#include <string>
#include <vector>
#include <base/Time.hpp>

namespace transformer
{
    /** Structure used in the TransformerStatus structure to report information
     * about specific transformations
     */
    struct TransformationStatus
    {
        /** The local (non-mapped) name of the source frame of this
         * transformation
         */
        std::string source_local;
        /** The local (non-mapped) name of the target frame of this
         * transformation
         */
        std::string target_local;
        /** The global (mapped) name of the source frame of this
         * transformation
         */
        std::string source_global;
        /** The global (mapped) name of the target frame of this
         * transformation
         */
        std::string target_global;

        /** The timestamp of the last generated transformation */
        base::Time last_generated_value;

        /** The number of chain elements that are used to generate this
         * transformation. 0 if no chain could be found so far.
         */
        int32_t chain_length;
        /** The number of transformations that have successfully been generated
         * so far
         */
        uint64_t generated_transformations;
        /** The number of time a transformation has been requested but could not
         * be generated because a chain was not yet found
         */
        uint64_t failed_no_chain;
        /** The number of time a transformation has been requested but could not
         * be generated because one of the dynamic transformations had simply no
         * information so far
         */
        uint64_t failed_no_sample;
        /** The number of time a transformation has been requested but could not
         * be generated because the needed information to interpolate was not
         * available
         */
        uint64_t failed_interpolation_impossible;

        TransformationStatus()
            : chain_length(0)
            , generated_transformations(0)
            , failed_no_chain(0)
            , failed_no_sample(0)
            , failed_interpolation_impossible(0) {}
    };
    
    /** 
     * Report of status for all transformations registered
     * in the transformer
     */
    struct TransformerStatus
    {
        base::Time time;
        std::vector<transformer::TransformationStatus> transformations;
    };
}

#endif

