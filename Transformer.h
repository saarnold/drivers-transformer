#ifndef TRANSFORMER_H
#define TRANSFORMER_H

#include <Eigen/Geometry>
#include <string>
#include <base/time.h>
#include <StreamAligner.hpp>
#include <map>
#include <boost/bind.hpp>

namespace transformer {
 
class Transformation
{
    public:
	base::Time time;
	Eigen::Transform3d transform;
	std::string from;
	std::string to;
};

/**
 * This a base class, that represens an abstract transformation from sourceFrame to targetFrame. 
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
	virtual bool getTransformation(const base::Time &atTime, bool doInterpolation, Eigen::Transform3d &tr) = 0;

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
	StaticTransformationElement(const std::string &sourceFrame, const std::string &targetFrame, const Eigen::Transform3d &transform) : TransformationElement(sourceFrame, targetFrame), staticTransform(transform) {};
	
	virtual bool getTransformation(const base::Time& atTime, bool doInterpolation, Eigen::Transform3d& tr)
	{
	    tr = staticTransform;
	    return true;
	};
	
    private:
	Eigen::Transform3d staticTransform;
};

/**
 * This class represents a dynamic transformation
 * 
 * 
 * */
class DynamicTransformationElement : public TransformationElement {
    public:
	DynamicTransformationElement(const std::string& sourceFrame, const std::string& targetFrame, aggregator::StreamAligner& aggregator, base::Time period);
	~DynamicTransformationElement();
	
	virtual bool getTransformation(const base::Time& atTime, bool doInterpolation, Eigen::Transform3d& result);

	int getStreamIdx() const
	{
	    return streamIdx;
	}
	
    private:
	
	void aggregatorCallback(const base::Time &ts, const Transformation &value); 

	aggregator::StreamAligner &aggregator;
	base::Time lastTransformTime;
	Transformation lastTransform;
	bool gotTransform;
	int streamIdx;
};

/**
 * This class represents an inverse transformation.
 * */
class InverseTransformationElement : public TransformationElement {
    public:
	InverseTransformationElement(TransformationElement *source): TransformationElement(source->getTargetFrame(), source->getSourceFrame()), nonInverseElement(source) {};
	virtual bool getTransformation(const base::Time& atTime, bool doInterpolation, Eigen::Transform3d& tr);
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
 * Base class for the TransformationMaker
 * 
 * This class is needed, so that templated version of
 * it can be stored in an array of pointers
 * */
class TransformationMakerBase
{
    public:
	TransformationMakerBase(const std::string &sourceFrame, const std::string &targetFrame) : sourceFrame(sourceFrame), targetFrame(targetFrame) { }
	
	/**
	 * This function sets a new transformationChain
	 * */
	void setTransformationChain(const std::vector<TransformationElement *> &chain)
	{
	    transformationChain = chain;
	}
	
	/**
	 * returns the souce frame
	 * */
	const std::string &getSourceFrame()
	{
	    return sourceFrame;
	}
	
	/**
	 * returns the target frame
	 * */
	const std::string &getTargetFrame()
	{
	    return targetFrame;
	}	
	
    protected:
	
	/**
	 * This functions tries to return a transformation from sourceFrame to targetFrame at the given time.
	 * 
	 * If no chain, or no transformation samples are available the function will return false
	 * Else it will return true an store the requested transformation in tr
	 * */
	bool getTransformation(const base::Time& time, transformer::Transformation& tr, bool doInterpolation);
	
	/**
	 * Transformation chain from sourceFrame to targetFrame
	 * */
	std::vector<TransformationElement *> transformationChain;
	std::string sourceFrame;
	std::string targetFrame;    
};

/**
 * Helper class, that will provide convenience callbacks 
 * */
template <class T> class TransformationMaker: public TransformationMakerBase {
    friend class Transformer;
    private:
	boost::function<void (const base::Time &ts, const T &value, const Transformation &t)> callback;
	
	/**
	 * Callback that is registered at the aggregator. 
	 * 
	 * This class will try to get the coresponding transformation 
	 * for the sample 'value' at time 'ts' and the call the convenience callback.
	 * 
	 * Note the convenience callback will not be called if no transformation is available
	 * */
	void aggregatorCallback(base::Time ts, T value) 
	{
	    Transformation transformation;
	    if(!getTransformation(ts, transformation, doInterpolation))
	    {
		std::cout << "Warning : dropping sample, as no transformation chain is available (yet)" << std::endl;
		return;
	    }
	    
	    //calls callback to pass result
	    callback(ts, value, transformation);
	}

	bool doInterpolation;
    public:
	
	/**
	 * Default constructor
	 * 
	 * Arguments:
	 * @param callback Pointer to the callback that should be called as soon as a sample and a transform is available
	 * @param sourceFrame Frame of the sample value
	 * @param targetFrame Frame of the result of the transformation t
	 * @param interpolate flag if interpolation should be done
	 * */
	TransformationMaker(boost::function<void (const base::Time &ts, const T &value, const Transformation &t)> callback, const std::string &sourceFrame, const std::string &targetFrame, bool interpolate): TransformationMakerBase(sourceFrame, targetFrame), callback(callback), doInterpolation(interpolate) {};
	
};

/**
 * A class that provides transformations to given samples, ordered in time.
 * */
class Transformer
{
    private:
	aggregator::StreamAligner aggregator;
	std::map<std::pair<std::string, std::string>, int> transformToStreamIndex;
	std::vector<TransformationMakerBase *> transformationMakers;
	TransformationTree transformationTree;
	bool locked;
	base::Time maxPeriod;
	
    public:
	Transformer() : locked(false) {};
	
	/**
	 * This function may be used to manually set a transformation chain.
	 * 
	 * The function seeks through all registered sample streams, and sets the given 
	 * chain to those streams, where the source and target frame matches.
	 * */
	void addTransformationChain(std::string from, std::string to, const std::vector<TransformationElement *> &chain);
	
	/**
	 * This function registes a new data stream together with an callback. 
	 * */
	template <class T> int registerDataStream(base::Time dataPeriod, std::string dataFrame, std::string targetFrame, boost::function<void (const base::Time &ts, const T &value, const Transformation &t)> callback, bool interpolate)
	{
	    if(locked)
		throw std::runtime_error("Tried to register data stream, after adding dynamic transformations");
	    
	    if(maxPeriod < dataPeriod)
		maxPeriod = dataPeriod;
	    
	    TransformationMaker<T> *trMaker = new TransformationMaker<T>(callback, dataFrame, targetFrame, interpolate);
	    transformationMakers.push_back(trMaker);
	    return aggregator.registerStream<T>(boost::bind( &TransformationMaker<T>::aggregatorCallback , trMaker, _1, _2 ), 0, dataPeriod);
	};
	
	/**
	 * Overloaded function, for Transformations, for convenience.
	 * 
	 * calls pushDynamicTransformation interally.
	 * */
	void pushData( int idx, base::Time ts, const Transformation& data )
	{
	    pushDynamicTransformation(data);
	};
	
	/** Push new data into the stream
	 * @param ts - the timestamp of the data item
	 * @param data - the data added to the stream
	 */
	template <class T> void pushData( int idx, base::Time ts, const T& data )
	{
	    aggregator.push(idx, ts, data);
	};
	
	/**
	 * Process data streams, this basically calls StreamAligner::step().
	 * */
	int step() {
	    return aggregator.step();
	}
	
	/**
	 * Function for adding new Transformation samples.
	 *
	 * This function will interally keep track of available 
	 * transformations and register streams for 'new'
	 * transformations.  
	 * */
	void pushDynamicTransformation(const Transformation &tr);
	
	/**
	 * Function for adding static Transformations.
	 * */
	void pushStaticTransformation(const Transformation &tr);

	
	/**
	 * Destructor, deletes the TransformationMakers
	 * */
	~Transformer();
};


}
#endif // TRANSFORMER_H
