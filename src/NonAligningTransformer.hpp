#ifndef NONALIGNINGTRANSFORMER_H
#define NONALIGNINGTRANSFORMER_H

#include "Transformer.hpp"

namespace transformer
{
class NonAlignedDynamicTransformationElement : public TransformationElement
{
public:
    NonAlignedDynamicTransformationElement(const std::string& sourceFrame, const std::string& targetFrame);
    virtual bool getTransformation(const base::Time& atTime, bool doInterpolation, TransformationType& result);

    void setTransformation(const base::Time& atTime, const TransformationType& tr);
    virtual void setTransformationChangedCallback(boost::function<void (const base::Time &ts)> callback)
    {
	elementChangedCallback = callback;
    };

private:
    boost::function<void (const base::Time &ts)> elementChangedCallback;
    base::Time lastTransformTime;
    TransformationType lastTransform;
    bool gotTransform;

};
    
    
class NonAligningTransformer : public transformer::Transformer
{
public:
    NonAligningTransformer();
    
    virtual void clear();
    
    virtual void pushDynamicTransformation(const TransformationType& tr);
    
private:
    std::map<std::pair<std::string, std::string>, NonAlignedDynamicTransformationElement *> transformToElementMap;

};

};

#endif // NONALIGNINGTRANSFORMER_H
