# bst_ipc

## bst ipc overview
 Inter-processor communication play a fundamental role in the message transformation between internal cores. As implemented in *ipc_send()*, *ipc_recv()*, bstipc copies data and uses interrupt to transfer data to other cores.  Let's explore these function in more details.

### Message passing
 asynchronous message passing
 A thread that does a *ipc_send()* to another thread on the other cores will not be blocked.
 
## ipc protocol definition
 1. bst ipc contains three layers.

## ipc recommending demo
 1. normal usage
    