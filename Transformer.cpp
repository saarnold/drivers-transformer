#include "Transformer.h"
#include <Eigen/LU>
#include <Eigen/SVD>
#include <assert.h>

namespace transformer {
    

class TransformationNode {
    public:
	TransformationNode() : parent(NULL) {};
	TransformationNode(const std::string &frameName, TransformationNode *parent, TransformationElement *parentToCurNode) : frameName(frameName), parent(parent), parentToCurNode(parentToCurNode) {};
	
	std::string frameName;
	TransformationNode *parent;
	TransformationElement *parentToCurNode;
	std::vector<TransformationNode *> childs;
	~TransformationNode() {
	    //delete all known childs
	    for(std::vector<TransformationNode *>::iterator it = childs.begin(); it != childs.end(); it++) {
		delete *it;
	    }
	    childs.clear();
	}
};

TransformationTree::~TransformationTree()
{
    for(std::vector<TransformationElement *>::iterator it = availableElements.begin(); it != availableElements.end(); it++)
    {
	delete *it;
    }
    availableElements.clear();
}

void TransformationTree::addTransformation(TransformationElement* element)
{
    //add transformation
    availableElements.push_back(element);
    
    //and it's inverse
    TransformationElement *inverse = new InverseTransformationElement(element);
    availableElements.push_back(inverse);
    
}

void TransformationTree::addMatchingTransforms(std::string from, TransformationNode *node)
{
    for(std::vector< TransformationElement* >::const_iterator it = availableElements.begin(); it != availableElements.end(); it++)
    {
	if((*it)->getSourceFrame() == from)
	{
	    //security check for not building A->B->A->B loops
	    if(node->parent && node->parent->frameName == (*it)->getTargetFrame())
		continue;
	    
	    node->childs.push_back(new TransformationNode((*it)->getTargetFrame(), node, *it));
	}	
    }
}

std::vector< TransformationNode* >::const_iterator TransformationTree::checkForMatchingChildFrame(const std::string& to, const transformer::TransformationNode& node)
{
    for(std::vector<TransformationNode *>::const_iterator it = node.childs.begin(); it != node.childs.end(); it++)
    {
	if((*it)->frameName == to)
	    return it;
    }
    
    return node.childs.end();
}


bool TransformationTree::getTransformationChain(std::string from, std::string to, std::vector< TransformationElement* >& result)
{
    TransformationNode node(from, NULL, NULL);
    
    std::vector<TransformationNode *> curLevel;
    curLevel.push_back(&node);
    
    for(int i = 0; i < maxSeekDepth && curLevel.size(); i++) {
	std::vector<TransformationNode *> nextLevel;

	for(std::vector<TransformationNode *>::iterator it = curLevel.begin(); it != curLevel.end(); it++)
	{
	    //expand tree node
	    addMatchingTransforms((*it)->frameName, *it);
	    
	    //check if a child of the node matches the wanted frame
	    std::vector< TransformationNode* >::const_iterator candidate = checkForMatchingChildFrame(to, **it);
	    if(candidate != (*it)->childs.end())
	    {
		std::cout << "Found Transformation chain from " << from << " to " << to << std::endl << "Chain is (reverse) : " ;
		
		TransformationNode *curNode = *candidate;
		result.reserve(i + 1);
		
		//found a valid transformation
		while(curNode->parent)
		{
		    result.push_back(curNode->parentToCurNode);
		    std::cout << " " << curNode->frameName << " " << curNode->parentToCurNode->getTargetFrame() << "<->" << curNode->parentToCurNode->getSourceFrame();
		    
		    curNode = curNode->parent;
		}
		std::cout << " " << curNode->frameName << std::endl;
		
		return true;
	    }
	    
	    //add childs of current level to search area for next level
	    nextLevel.insert(nextLevel.end(), (*it)->childs.begin(), (*it)->childs.end());
	}
	
	curLevel = nextLevel;
    }
    
    std::cout << "could not find result for " << from << " " << to << std::endl;

    return false;
}

bool InverseTransformationElement::getTransformation(const base::Time& atTime, bool doInterpolation, transformer::Transformation& tr)
{
    if(nonInverseElement->getTransformation(atTime, doInterpolation, tr)){
	Eigen::Transform3d tr2(Eigen::Transform3d::Identity());
	tr2 = tr;
	tr2 = tr2.inverse();
	tr.setTransform(tr2);
	return true;
    }
    return false;
};

DynamicTransformationElement::DynamicTransformationElement(const std::string& sourceFrame, const std::string& targetFrame, aggregator::StreamAligner& aggregator): TransformationElement(sourceFrame, targetFrame), aggregator(aggregator), gotTransform(false) 
{
    //giving a buffersize of zero means no buffer limitation at all
    //giving a period of zero means, block until next sample is available
    streamIdx = aggregator.registerStream<Transformation>(boost::bind( &transformer::DynamicTransformationElement::aggregatorCallback , this, _1, _2 ), 0, base::Time(), -10);
}

DynamicTransformationElement::~DynamicTransformationElement()
{
    //FIXME, get jakob and figure out, how to remove streams
}

void DynamicTransformationElement::aggregatorCallback(const base::Time& ts, const transformer::Transformation& value)
{
    std::cout << "DynamicTransformationElement callback time : " << ts << std::endl;
    gotTransform = true;
    lastTransform = value;
    lastTransformTime = ts;
}

bool DynamicTransformationElement::getTransformation(const base::Time& atTime, bool doInterpolation, transformer::Transformation& result)
{
    if(!gotTransform)
    {
	std::cout << "no sample available yet" << std::endl;
	//no sample available, return
	return false;
    }
    
    std::cout << "Cur sample time " << lastTransformTime << std::endl;
    
    if(doInterpolation)
    {
	std::pair<base::Time, Transformation> next_sample;
	
	if(!aggregator.getNextSample(streamIdx, next_sample))
	{
	    std::cout << "could not get next sample" << std::endl;
	    //not enought samples for itnerpolation available
	    return false;
	}
	
	double timeForward = (atTime - lastTransformTime).toSeconds();
	
	if(timeForward < 0) 
	{
	    throw std::runtime_error("Error, time of sample is lower than transformation time"); 
	}
	
	if(timeForward == 0) 
	{
	    //transform time is equal to sample time, no interpolation needed
	    result = lastTransform;

	    return true;
	}
	
	Transformation interpolated;
	interpolated.initSane();

	double timeBetweenTransforms = (next_sample.first - lastTransformTime).toSeconds();

	assert(timeBetweenTransforms > timeForward);
	
	double factor = timeForward / timeBetweenTransforms;
	
	Eigen::Quaterniond start_r(lastTransform.orientation);
	Eigen::Quaterniond end_r(next_sample.second.orientation);
	
	interpolated.orientation = (start_r.slerp(factor, end_r));
	
	Eigen::Vector3d start_t(lastTransform.position);
	Eigen::Vector3d end_t(next_sample.second.position);
	
	interpolated.position = start_t + (end_t - start_t) * factor; 

	result = interpolated;
    } else {
	result = lastTransform;
    }

    return true;
};

bool TransformationMakerBase::getTransformation(const base::Time &time, Transformation& tr, bool doInterpolation)
{
    tr.initSane();
    tr.sourceFrame = sourceFrame;
    tr.targetFrame = targetFrame;
    tr.time = time;

    Eigen::Transform3d fullTransformation(Eigen::Transform3d::Identity());

    if(transformationChain.empty()) 
    {
	return false;
    }
    
    for(std::vector<TransformationElement *>::const_iterator it = transformationChain.begin(); it != transformationChain.end(); it++)
    {
	Transformation tr;
	if(!(*it)->getTransformation(time, doInterpolation, tr))
	{
	    //no sample available, return
	    return false;
	}
	
	//TODO, this might be a costly operation
	Eigen::Transform3d trans = tr;
	
	//apply transformation
	fullTransformation = fullTransformation * trans;
    }

    
    tr.setTransform(fullTransformation);
    return true;
}

bool TransformationMakerBase::getTransformationChain(const base::Time& time, std::vector< Transformation >& tr, bool doInterpolation)
{
    if(transformationChain.empty()) 
    {
	return false;
    }

    tr.resize(transformationChain.size());

    Transformation transform;
    
    int i = 0;
    
    for(std::vector<TransformationElement *>::const_iterator it = transformationChain.begin(); it != transformationChain.end(); it++)
    {
	tr[i].sourceFrame = (*it)->getSourceFrame();
	tr[i].targetFrame = (*it)->getTargetFrame();
	tr[i].time = time;
	if(!(*it)->getTransformation(time, doInterpolation, tr[i]))
	{
	    //no sample available, return
	    return false;
	}
	
	i++;
    }

    return true;
}

bool Transformer::getTransformation(const std::string& sourceFrame, const std::string& targetFrame, const base::Time& atTime, bool interpolate, transformer::Transformation &result)
{
    //TODO faster seek
    //look for existing chain, that matches our request
    for(std::vector<TransformationMakerBase *>::iterator streams = transformationMakers.begin(); streams != transformationMakers.end(); streams++)
    {
	if((*streams)->getSourceFrame() == sourceFrame && (*streams)->getTargetFrame() == targetFrame)
	{
	    return (*streams)->getTransformation(atTime, result, interpolate);
	}
    }

    //no chain there, try to create a new one 
    std::vector< TransformationElement* > trChain;
    if(transformationTree.getTransformationChain(sourceFrame, targetFrame, trChain))
    {
	std::cout << "Setting tr chain " << std::endl;
	TransformationMakerBase *tmb = new TransformationMakerBase(sourceFrame, targetFrame);
	tmb->setTransformationChain(trChain);
	
	transformationMakers.push_back(tmb);
	
	return tmb->getTransformation(atTime, result, interpolate);
    }

    //no chain available
    return false;
}


void Transformer::pushDynamicTransformation(const transformer::Transformation& tr)
{
    std::map<std::pair<std::string, std::string>, int>::iterator it = transformToStreamIndex.find(std::make_pair(tr.sourceFrame, tr.targetFrame));
    
    //we got an unknown transformation
    if(it == transformToStreamIndex.end()) {

	//create a representation of the dynamic transformation
	DynamicTransformationElement *dynamicElement = new DynamicTransformationElement(tr.sourceFrame, tr.targetFrame, aggregator);
	
	int streamIdx = dynamicElement->getStreamIdx();
	
	transformToStreamIndex[std::make_pair(tr.sourceFrame, tr.targetFrame)] = streamIdx;
	
	std::cout << "Registering new stream for transformation from " << tr.sourceFrame << " to " << tr.targetFrame << " index is " << streamIdx << std::endl;
	
	//add new dynamic element to transformation tree
	transformationTree.addTransformation(dynamicElement);
	
	//seek through all available data streams and update transformation chains
	for(std::vector<TransformationMakerBase *>::iterator streams = transformationMakers.begin(); streams != transformationMakers.end(); streams++)
	{
	    std::vector< TransformationElement* > trChain;
	    
	    if(transformationTree.getTransformationChain((*streams)->getSourceFrame(), (*streams)->getTargetFrame(), trChain))
	    {
		std::cout << "Setting tr chain " << std::endl;
		(*streams)->setTransformationChain(trChain);
	    }
	}
	
	it = transformToStreamIndex.find(std::make_pair(tr.sourceFrame, tr.targetFrame));
	assert(it != transformToStreamIndex.end());
    }
    
    //push sample
    aggregator.push(it->second, tr.time, tr);
}

void Transformer::pushStaticTransformation(const transformer::Transformation& tr)
{
    transformationTree.addTransformation(new StaticTransformationElement(tr.sourceFrame, tr.targetFrame, tr));
}
    
void Transformer::addTransformationChain(std::string from, std::string to, const std::vector< TransformationElement* >& chain)
{
    for(std::vector<TransformationMakerBase *>::iterator it = transformationMakers.begin();
	it != transformationMakers.end(); it++) 
    {
	if((*it)->getSourceFrame() == from && (*it)->getTargetFrame() == to)
	{
	    (*it)->setTransformationChain(chain);
	}
    }
}
    
    
Transformer::~Transformer()
{
    for(std::vector<TransformationMakerBase *>::iterator it = transformationMakers.begin(); it != transformationMakers.end(); it++)
    {
	delete *it;
    }
    transformationMakers.clear();
}
    
}