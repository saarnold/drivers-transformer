#ifndef TRANSFORMER_H
#define TRANSFORMER_H

#include <Eigen/Geometry>
#include <string>
#include <base/time.h>
#include <StreamAligner.hpp>
#include <map>
#include <boost/bind.hpp>
#include <base/samples/rigid_body_state.h>

namespace transformer {
 
typedef base::samples::RigidBodyState TransformationType;
class TransformationElement;

class Transformation
{
    friend class Transformer;
    private:
	Transformation(const std::string &sourceFrame, const std::string &targetFrame) : sourceFrame(sourceFrame), targetFrame(targetFrame) {};
	std::string sourceFrame;
	std::string targetFrame;
	std::string sourceFrameMapped;
	std::string targetFrameMapped;
	std::vector<TransformationElement *> transformationChain;

	void setFrameMapping(const std::string &frameName, const std::string &newName);
	
	/**
	 * This function sets a new transformationChain
	 * */
	void setTransformationChain(const std::vector<TransformationElement *> &chain)
	{
	    transformationChain = chain;
	}

    public:
	Transformation(const Transformation &other)
	{
	}
	
	/**
	 * returns the souce frame
	 * */
	const std::string &getSourceFrame()
	{
	    if(sourceFrameMapped.empty())
		return sourceFrame;
	    
	    return sourceFrameMapped;
	}
	
	/**
	 * returns the target frame
	 * */
	const std::string &getTargetFrame()
	{
	    if(targetFrameMapped.empty())
		return targetFrame;
	    
	    return targetFrameMapped;
	}	
	
	/**
	 * This functions tries to return the transformation from sourceFrame to targetFrame at the given time.
	 * 
	 * If no chain, or no transformation samples are available the function will return false
	 * Else it will return true and store the requested transformation in @param result
	 * */
	bool get(const base::Time& atTime, transformer::TransformationType& result, bool interpolate) const;
	bool getChain(const base::Time& atTime, std::vector<TransformationType>& result, bool interpolate) const;
};

/**
 * This is a base class, that represens an abstract transformation from sourceFrame to targetFrame. 
 * */
class TransformationElement {
    public:
	TransformationElement(const std::string &sourceFrame, const std::string &targetFrame): sourceFrame(sourceFrame), targetFrame(targetFrame) {};

	/**
	 * This method my be asked for a concrete transformation at a time X.
	 * On request this should should interpolate the transformation to
	 * match the given time better than the available data. 
	 * 
	 * Note if no transformation is available getTransformation will return false
	 * */
	virtual bool getTransformation(const base::Time &atTime, bool doInterpolation, TransformationType &tr) = 0;

	/**
	 * returns the name of the source frame
	 * */
	const std::string &getSourceFrame()
	{
	    return sourceFrame;
	}
	
	/**
	 * returns the name of the target frame
	 * */
	const std::string &getTargetFrame()
	{
	    return targetFrame;
	}

    private:
	std::string sourceFrame;
	std::string targetFrame;
};

/**
 * This class represents a static transformation
 * */
class StaticTransformationElement : public TransformationElement {
    public:
	StaticTransformationElement(const std::string &sourceFrame, const std::string &targetFrame, const TransformationType &transform) : TransformationElement(sourceFrame, targetFrame), staticTransform(transform) {};
	
	virtual bool getTransformation(const base::Time& atTime, bool doInterpolation, TransformationType& tr)
	{
	    tr = staticTransform;
	    return true;
	};
	
    private:
	TransformationType staticTransform;
};

/**
 * This class represents a dynamic transformation
 * 
 * 
 * */
class DynamicTransformationElement : public TransformationElement {
    public:
	DynamicTransformationElement(const std::string& sourceFrame, const std::string& targetFrame, aggregator::StreamAligner& aggregator);
	~DynamicTransformationElement();
	
	virtual bool getTransformation(const base::Time& atTime, bool doInterpolation, TransformationType& result);

	int getStreamIdx() const
	{
	    return streamIdx;
	}
	
    private:
	
	void aggregatorCallback(const base::Time &ts, const TransformationType &value); 

	aggregator::StreamAligner &aggregator;
	base::Time lastTransformTime;
	TransformationType lastTransform;
	bool gotTransform;
	int streamIdx;
};

/**
 * This class represents an inverse transformation.
 * */
class InverseTransformationElement : public TransformationElement {
    public:
	InverseTransformationElement(TransformationElement *source): TransformationElement(source->getTargetFrame(), source->getSourceFrame()), nonInverseElement(source) {};
	virtual bool getTransformation(const base::Time& atTime, bool doInterpolation, TransformationType& tr);
    private:
	TransformationElement *nonInverseElement;
};


class  TransformationNode;

/**
 * A class that can be used to get a transformation chain from a set of TransformationElements
 * */
class TransformationTree
{
    public:
	///default constructor
	TransformationTree() : maxSeekDepth(20) {};
	
	/**
	 * Adds a TransformationElement to the set of available elements.
	 * 
	 * Note, internal the TransformationTree will also add an inverse
	 * transformation.
	 * */
	void addTransformation(TransformationElement *element);
	
	/**
	 * This function tries to generate a transformationChain from 'from' to 'to'.
	 * 
	 * To generate the transformationChain this function will spann a tree of
	 * transformations originating from 'from'. The function will then perform a
	 * breadth-first search until it either finds a chain, the tree can't be expanded
	 * further, or the search depth is deeper than maxSeekDepth.
	 * 
	 * In case a chain was found the function returns true and the chain is stored in result.
	 * */
	bool getTransformationChain(std::string from, std::string to, std::vector<TransformationElement *> &result);
	
	/**
	 * Destructor, deletes all TransformationElements saved in availableElements
	 * */
	~TransformationTree();	
	
    private:
	///maximum seek depth while trying to find a transformationChain
	const int maxSeekDepth;

	/**
	 * This function will expand the given node.
	 * 
	 * To expand the node, the function will look up all available TransformationElements
	 * add new node for those, where the sourceFrame matches from
	 * */
	void addMatchingTransforms(std::string from, transformer::TransformationNode* node);
	
	/**
	 * Seeks through childs of the node and looks for a node that has frame 'to'
	 * 
	 * If found the function returns an iterator the the child.
	 * If not returns an iterator pointing to node.childs.end()
	 * */
	std::vector< TransformationNode* >::const_iterator checkForMatchingChildFrame(const std::string &to, const TransformationNode &node);
	
	/// List of available transformation elements
	std::vector<TransformationElement *> availableElements;
};

/**
 * A class that provides transformations to given samples, ordered in time.
 * */
class Transformer
{
    private:
	aggregator::StreamAligner aggregator;
	std::map<std::pair<std::string, std::string>, int> transformToStreamIndex;
	std::vector<Transformation *> transformations;
	TransformationTree transformationTree;

	void recomputeAvailableTransformations();
	
    public:
	
	Transformer() {};
	
	/**
	 * This function may be used to manually set a transformation chain.
	 * 
	 * The function seeks through all registered sample streams, and sets the given 
	 * chain to those streams, where the source and target frame matches.
	 * */
	void addTransformationChain(std::string from, std::string to, const std::vector<TransformationElement *> &chain);
	
	/**
	 * This function registers a wanted transformation at the transformation stack.
	 * 
	 * It returns a reference to an object that represents the wanted transformation.
	 * 
	 * Note, the time of the transformation advances if one calls step() on the
	 * transformer.
	 * */
	Transformation &registerTransformation(std::string sourceFrame, std::string targetFrame);
	
	/**
	 * Registers a callback that will be called every time a new transformation is available 
	 * for the given Transformation handle.
	 * FIXME The callback will only be called after requestTransformationAtTime for the 
	 * returned stream. 
	 * */
	int registerTransfromCallback(const Transformation &transform , boost::function<void (const base::Time &ts, const Transformation &t)> callback) 
	{
	    return aggregator.registerStream<bool>(boost::bind( callback, _1, boost::ref(transform) ), 0, base::Time());
	}

	/**
	 * This function registes a new data stream together with an callback. 
	 * 
	 * The callback will be called every time a new data sample is available.
	 * */
	template <class T> int registerDataStream(base::Time dataPeriod, boost::function<void (const base::Time &ts, const T &value)> callback)
	{
	    return aggregator.registerStream<T>(boost::bind( callback, _1, _2), 0, dataPeriod);
	};

	/**
	 * This function registes a new data stream together with an callback. 
	 * 
	 * The callback will be called every time a new data sample is available.
	 * */
	template <class T> int registerDataStreamWithTransform(base::Time dataPeriod, Transformation &transformation, boost::function<void (const base::Time &ts, const T &value, const Transformation &t)> callback)
	{
	    return aggregator.registerStream<T>(boost::bind( callback, _1, _2, boost::ref(transformation) ), 0, dataPeriod);
	};

	/**
	 * Overloaded function, for Transformations, for convenience.
	 * 
	 * calls pushDynamicTransformation interally.
	 * */
	void pushData( int idx, base::Time ts, const TransformationType& data )
	{
	    pushDynamicTransformation(data);
	};
	
	void requestTransformationAtTime(int idx, base::Time ts)
	{
	    aggregator.push(idx, ts, false);
	};
	
	/** Push new data into the stream
	 * @param ts - the timestamp of the data item
	 * @param data - the data added to the stream
	 */
	template <class T> void pushData( int idx,const base::Time &ts, const T& data )
	{
	    aggregator.push(idx, ts, data);
	};
	
	/**
	 * Process data streams, this basically calls StreamAligner::step().
	 * */
	int step() {
	    return aggregator.step();
	}
	
	void setTimeout(const base::Time &t )
	{
	    aggregator.setTimeout(t);
	}
	
	/**
	 * Function for adding new Transformation samples.
	 *
	 * This function will interally keep track of available 
	 * transformations and register streams for 'new'
	 * transformations.  
	 * */
	void pushDynamicTransformation(const TransformationType &tr);
	
	/**
	 * Function for adding static Transformations.
	 * */
	void pushStaticTransformation(const TransformationType &tr);

	
	void setFrameMapping(const std::string &frameName, const std::string &newName);
	
	
	/**
	 * Destructor, deletes the TransformationMakers
	 * */
	~Transformer();
};


}
#endif // TRANSFORMER_H
