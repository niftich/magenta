// Copyright 2016 The Fuchsia Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#pragma once

#include <magenta/assert.h>
#include <mxtl/intrusive_container_utils.h>
#include <mxtl/intrusive_pointer_traits.h>
#include <mxtl/intrusive_wavl_tree_internal.h>
#include <mxtl/macros.h>

// Implementation Notes:
//
// WAVLTree<> is an implementation of a "Weak AVL" tree; a self
// balancing binary search tree whose rebalancing algorithm was
// originally described in
//
// Bernhard Haeupler, Siddhartha Sen, and Robert E. Tarjan. 2015.
// Rank-Balanced Trees. ACM Trans. Algorithms 11, 4, Article 30 (June 2015), 26 pages.
// DOI=http://dx.doi.org/10.1145/2689412
//
// See also
// https://en.wikipedia.org/wiki/WAVL_tree
// http://www.cs.princeton.edu/~sssix/papers/rb-trees-talg.pdf
//
// WAVLTree<>s, like HashTables, are associative containers and support all of
// the same key-centric operations (such as find() and insert_or_find()) that
// HashTables support.
//
// Additionally, WAVLTree's are internally ordered by key (unlike HashTables
// which are un-ordered).  Iteration from begin() to end() runs in amortized
// constant time and will enumerate the elements in monotonically increasing
// order (as defined by the KeyTraits::LessThan operation).
//
// Two additional operations are supported because of the ordered nature of a
// WAVLTree:
// upper_bound(key) : Finds the element (E) in the tree such that E.key > key
// lower_bound(key) : Finds the element (E) in the tree such that E.key >= key
//
// The worst depth of a WAVL tree depends on whether or not the tree has ever
// been subject to erase operations.
// ++ If the tree has seen only insert operations, the worst case depth of the
//    tree is log_phi(N), where phi is the golden ratio.  This is the same bound
//    as that of an AVL tree.
// ++ If the tree has seen erase operations in addition to insert operations,
//    the worst case depth of the tree is 2*log_2(N).  This is the same bound as
//    a Red-Black tree.
//
// Insertion runs in O(log) time; finding the location takes O(log) time while
// post-insert rebalancing runs in amortized constant time.
//
// Erase-by-key runs in O(log) time; finding the node to erase takes O(log) time
// while post-erase rebalancing runs in amortized constant time.
//
// Because of the intrusive nature of the container, direct-erase operations
// (AKA, erase operations where the reference to the element to be erased is
// already known) run in amortized constant time.
//
namespace mxtl {

template <typename PtrType, typename RankType>
struct WAVLTreeNodeStateBase {
    using PtrTraits = internal::ContainerPtrTraits<PtrType>;

    typename PtrTraits::RawPtrType parent_ = nullptr;
    typename PtrTraits::PtrType    left_   = nullptr;
    typename PtrTraits::PtrType    right_  = nullptr;
    RankType rank_;

    bool IsValid() const     { return (parent_ || (!parent_ && !left_ && !right_)); }
    bool InContainer() const { return (parent_ != nullptr); }
};

template <typename PtrType>
struct WAVLTreeNodeState<PtrType, bool> : public WAVLTreeNodeStateBase<PtrType, bool> {
    bool rank_parity() const   { return this->rank_; }
    void promote_rank()        { this->rank_ = !this->rank_; }
    void double_promote_rank() { }
    void demote_rank()         { this->rank_ = !this->rank_; }
    void double_demote_rank()  { }
};

template <typename PtrType, typename RankType = bool>
struct DefaultWAVLTreeTraits {
    using PtrTraits = internal::ContainerPtrTraits<PtrType>;
    static WAVLTreeNodeState<PtrType, RankType>& node_state(typename PtrTraits::RefType obj) {
        return obj.wavl_node_state_;
    }
};

template <typename PtrType>
struct WAVLTreeContainable {
public:
    bool InContainer() const { return wavl_node_state_.InContainer(); }

private:
    friend DefaultWAVLTreeTraits<PtrType, bool>;
    WAVLTreeNodeState<PtrType, bool> wavl_node_state_;
};

template <typename _KeyType,
          typename _PtrType,
          typename _KeyTraits  = DefaultKeyedObjectTraits<
                                    _KeyType,
                                    typename internal::ContainerPtrTraits<_PtrType>::ValueType>,
          typename _NodeTraits = DefaultWAVLTreeTraits<_PtrType>,
          typename _Observer   = tests::intrusive_containers::DefaultWAVLTreeObserver>
class WAVLTree {
private:
    // Private fwd decls of the iterator implementation.
    template <typename IterTraits> class iterator_impl;
    struct iterator_traits;
    struct const_iterator_traits;

public:
    // Aliases used to reduce verbosity and expose types/traits to tests
    using KeyType       = _KeyType;
    using PtrType       = _PtrType;
    using KeyTraits     = _KeyTraits;
    using NodeTraits    = _NodeTraits;
    using Observer      = _Observer;
    using PtrTraits     = internal::ContainerPtrTraits<PtrType>;
    using RawPtrType    = typename PtrTraits::RawPtrType;
    using ValueType     = typename PtrTraits::ValueType;
    using ContainerType = WAVLTree<KeyType, PtrType, KeyTraits, NodeTraits, Observer>;
    using CheckerType   = ::mxtl::tests::intrusive_containers::WAVLTreeChecker;

    // Declarations of the standard iterator types.
    using iterator       = iterator_impl<iterator_traits>;
    using const_iterator = iterator_impl<const_iterator_traits>;

    // WAVL Trees support amortized constant erase.  Technically, the worst case
    // for any individual erase operation involves O(log) demotions, followed by
    // a double rotation operation.  Given D total erase operations, however,
    // the maximum number of operations (demotions + rotations) is 2*D, given
    // the amortized constant erase time.
    //
    static constexpr bool SupportsConstantOrderErase = true;
    static constexpr bool SupportsConstantOrderSize = true;
    static constexpr bool IsAssociative = true;
    static constexpr bool IsSequenced = false;

    // Default construction gives an empty tree.
    constexpr WAVLTree() { }

    // Rvalue construction is permitted, but will result in the move of the tree
    // contents from one instance of the list to the other (even for unmanaged
    // pointers)
    explicit WAVLTree(WAVLTree&& other_tree) {
        swap(other_tree);
    }

    // Rvalue assignment is permitted for managed trees, and when the target is
    // an empty tree of unmanaged pointers.  Like Rvalue construction, it will
    // result in the move of the source contents to the destination.
    WAVLTree& operator=(WAVLTree&& other_tree) {
        MX_DEBUG_ASSERT(PtrTraits::IsManaged || is_empty());

        clear();
        swap(other_tree);

        return *this;
    }

    ~WAVLTree() {
        // It is considered an error to allow a tree of unmanaged pointers to
        // destruct of there are still elements in it.  Managed pointer trees
        // will automatically release their references to their elements.
        MX_DEBUG_ASSERT(PtrTraits::IsManaged || is_empty());
        clear();
    }

    // Standard begin/end, cbegin/cend iterator accessors.
    iterator        begin()       { return iterator(left_most_); }
    const_iterator  begin() const { return const_iterator(left_most_); }
    const_iterator cbegin() const { return const_iterator(left_most_); }

    iterator          end()       { return iterator(sentinel()); }
    const_iterator    end() const { return const_iterator(sentinel()); }
    const_iterator   cend() const { return const_iterator(sentinel()); }

    // make_iterator : construct an iterator out of a pointer to an object
    iterator make_iterator(ValueType& obj) { return iterator(&obj); }

    // is_empty : True if the tree has at least one element in it, false otherwise.
    bool is_empty() const { return root_ == nullptr; }

    // front
    //
    // Return a reference to the element at the front of the list without
    // removing it.  It is an error to call front on an empty list.
    typename PtrTraits::RefType front() {
        MX_DEBUG_ASSERT(PtrTraits::IsValid(left_most_));
        return *left_most_;
    }

    typename PtrTraits::ConstRefType front() const {
        MX_DEBUG_ASSERT(PtrTraits::IsValid(left_most_));
        return *left_most_;
    }

    // back
    //
    // Return a reference to the element at the back of the list without
    // removing it.  It is an error to call back on an empty list.
    typename PtrTraits::RefType back() {
        MX_DEBUG_ASSERT(PtrTraits::IsValid(right_most_));
        return *right_most_;
    }

    typename PtrTraits::ConstRefType back() const {
        MX_DEBUG_ASSERT(PtrTraits::IsValid(right_most_));
        return *right_most_;
    }

    void insert(const PtrType& ptr) { insert(PtrType(ptr)); }
    void insert(PtrType&& ptr)      { internal_insert(mxtl::move(ptr)); }

    // insert or find
    //
    // Insert the object pointed to by ptr if it is not already in the
    // tree, or find the object that the ptr collided with instead.
    //
    // @param ptr A pointer to the new object to insert.
    // @param iter An optional out parameter pointer to an iterator which
    // will reference either the newly inserted item, or the item whose key
    // collided with ptr.
    //
    // @return true if there was no collision and the item was successfully
    // inserted.  false otherwise.
    //
    bool insert_or_find(const PtrType& ptr, iterator* iter = nullptr) {
        return insert_or_find(PtrType(ptr), iter);
    }

    bool insert_or_find(PtrType&& ptr, iterator* iter = nullptr) {
        RawPtrType obj = PtrTraits::GetRaw(ptr);
        RawPtrType collision = nullptr;

        internal_insert(mxtl::move(ptr), &collision);

        if (iter)
            *iter = collision ? iterator(collision) : iterator(obj);

        return !collision;
    }

    // pop_front and pop_back
    //
    // Removes either the left-most or right-most member of tree and transfers
    // the pointer to the caller.  If the list is empty, return a nullptr
    // instance of PtrType.
    PtrType pop_front() { return internal_erase(left_most_); }
    PtrType pop_back()  { return internal_erase(right_most_); }

    // find
    //
    // Find the first node in the tree whose key matches "key" and return an
    // iterator to it.  Return end() if no node in the tree has a key which
    // matches "key".
    const_iterator find(const KeyType& key) const {
        RawPtrType node = PtrTraits::GetRaw(root_);

        while (PtrTraits::IsValid(node)) {
            auto node_key = KeyTraits::GetKey(*node);

            if (KeyTraits::EqualTo(key, node_key))
                return const_iterator(node);

            auto& ns = NodeTraits::node_state(*node);
            node = PtrTraits::GetRaw(KeyTraits::LessThan(key, node_key) ? ns.left_ : ns.right_);
        }

        return end();
    }

    iterator find(const KeyType& key) {
        const_iterator citer = const_cast<const ContainerType*>(this)->find(key);
        return iterator(citer.node_);
    }

    // upper_bound
    //
    // Find the first node in the tree whose key is strictly greater than the
    // caller provided key.  Returns end() if no such node exists.
    const_iterator upper_bound(const KeyType& key) const {
        return internal_upper_lower_bound<UpperBoundTraits>(key);
    }

    iterator upper_bound(const KeyType& key) {
        const_iterator citer = const_cast<const ContainerType*>(this)->upper_bound(key);
        return iterator(citer.node_);
    }

    // lower_bound
    //
    // Find the first node in the tree whose key is greater than or equal to the
    // caller provided key.  Returns end() if no such node exists.
    const_iterator lower_bound(const KeyType& key) const {
        return internal_upper_lower_bound<LowerBoundTraits>(key);
    }

    iterator lower_bound(const KeyType& key) {
        const_iterator citer = const_cast<const ContainerType*>(this)->lower_bound(key);
        return iterator(citer.node_);
    }

    // erase
    //
    // Remove the first element in the tree whose key matches "key" and return a
    // pointer the removed object.  Return a nullptr instance of PtrType if no
    // such element exists in the tree.
    PtrType erase(const KeyType& key) { return erase(find(key)); }

    // erase (direct)
    //
    // Remove the object directly referenced either by "iter" or "obj" from the
    // tree and return a pointer to it.  In the case of an iterator based erase,
    // return a nullptr instance of PtrType if the iterator is invalid.  It is
    // an error to either use a valid iterator from a different tree instance,
    // or to attempt to remove an element which is not currently a member of
    // this tree instance.
    PtrType erase(const iterator& iter) {
        if (!iter.IsValid())
            return PtrType(nullptr);

        return internal_erase(&(*iter));
    }

    PtrType erase(ValueType& obj) {
        return internal_erase(&obj);
    }

    // clear
    //
    // Clear out the tree, unlinking all of the elements in the process.  For
    // managed pointer types, this will release all references held by the tree
    // to the objects which were in it.
    void clear() {
        if (is_empty())
            return;

        MX_DEBUG_ASSERT(PtrTraits::IsValid(root_));
        MX_DEBUG_ASSERT(PtrTraits::IsValid(left_most_));
        MX_DEBUG_ASSERT(PtrTraits::IsValid(right_most_));

        // Detach the left and right sentinels right now so that we don't have
        // to worry about them while cleaning up the tree.
        PtrTraits::DetachSentinel(NodeTraits::node_state(*left_most_).left_);
        PtrTraits::DetachSentinel(NodeTraits::node_state(*right_most_).right_);

        RawPtrType owner    = sentinel();
        PtrType*   link_ptr = &root_;

        while (true) {
            auto& ns = NodeTraits::node_state(**link_ptr);

            if ((ns.left_ == nullptr) && (ns.right_ == nullptr)) {
                // Leaf node.  Trim it.
                MX_DEBUG_ASSERT(ns.parent_ == owner);
                ns.parent_ = nullptr;
                *link_ptr  = nullptr;

                // If we are removing the root, and it is a leaf node, then we
                // are done.
                if (link_ptr == &root_)
                    break;

                // Now climb back up the tree.
                link_ptr = &GetLinkPtrToNode(owner);
                owner    = NodeTraits::node_state(*owner).parent_;
            } else {
                // Non-leaf node, descend.  We have already detached the left
                // and right sentinels, so we shouldn't be seeing any here.
                MX_DEBUG_ASSERT(!PtrTraits::IsSentinel(ns.left_));
                MX_DEBUG_ASSERT(!PtrTraits::IsSentinel(ns.right_));

                owner    = PtrTraits::GetRaw(*link_ptr);
                link_ptr = (ns.left_ != nullptr) ? &ns.left_ : &ns.right_;
            }
        }

        MX_DEBUG_ASSERT(root_ == nullptr);
        left_most_  = sentinel();
        right_most_ = sentinel();
        count_ = 0;
    }

    // clear_unsafe
    //
    // See comments in mxtl/intrusive_single_list.h
    // Think carefully before calling this!
    void clear_unsafe() {
        static_assert(PtrTraits::IsManaged == false,
                     "clear_unsafe is not allowed for containers of managed pointers");

        root_       = nullptr;
        left_most_  = sentinel();
        right_most_ = sentinel();
        count_      = 0;
    }

    // swap : swaps the contents of two trees.
    void swap(WAVLTree& other) {
        PtrTraits::Swap(root_, other.root_);
        pod_swap(left_most_,  other.left_most_);
        pod_swap(right_most_, other.right_most_);
        pod_swap(count_,      other.count_);

        // Fix up the sentinel values.
        FixSentinelsAfterSwap();
        other.FixSentinelsAfterSwap();
    }

    // size : return the current number of elements in the tree.
    size_t size() const { return count_; };

    // erase_if
    //
    // Find the first member of the list which satisfies the predicate given by
    // 'fn' and erase it from the list, returning a referenced pointer to the
    // removed element.  Return nullptr if no element satisfies the predicate.
    template <typename UnaryFn>
    PtrType erase_if(UnaryFn fn) {
        for (auto iter = begin(); iter != end(); ++iter) {
            if (fn(*iter))
                return erase(iter);
        }
        return PtrType(nullptr);
    }

    // find_if
    //
    // Find the first member of the list which satisfies the predicate given by
    // 'fn' and return a const& to the PtrType in the list which refers to it.
    // Return nullptr if no member satisfies the predicate.
    template <typename UnaryFn>
    const_iterator find_if(UnaryFn fn) const {
        for (auto iter = begin(); iter != end(); ++iter)
            if (fn(*iter))
                return const_iterator(iter.node_);

        return const_iterator(sentinel());
    }

    template <typename UnaryFn>
    iterator find_if(UnaryFn fn) {
        const_iterator citer = const_cast<const ContainerType*>(this)->find_if(fn);
        return iterator(citer.node_);
    }

private:
    // The traits of a non-const iterator
    struct iterator_traits {
        using RefType    = typename PtrTraits::RefType;
        using RawPtrType = typename PtrTraits::RawPtrType;
    };

    // The traits of a const iterator
    struct const_iterator_traits {
        using RefType    = typename PtrTraits::ConstRefType;
        using RawPtrType = typename PtrTraits::ConstRawPtrType;
    };

    // Trait classes used to help implement symmetric operations.
    //
    // Notes about notation:
    // + LR denotes Left for the Forward version of the operation and Right
    //   for the Reverse version.
    // + RL denotes Right for the Forward version of the operation and Left
    //   for the Reverse version.
    //
    // Examples...
    // Forward : LR-child of node X == the left child of node X.
    // Reverse : LR-child of node X == the right child of node X.
    //
    // Forward : RL-most node of the tree == the right most node of the tree
    // Reverse : RL-most node of the tree == the left most node of the tree
    struct ReverseTraits;   // fwd decl
    struct ForwardTraits {
        using Inverse = ReverseTraits;

        template <typename NodeState> static PtrType& LRChild(NodeState& ns) { return ns.left_;  }
        template <typename NodeState> static PtrType& RLChild(NodeState& ns) { return ns.right_; }

        template <typename NodeState>
        static RawPtrType LRRawChild(NodeState& ns) { return PtrTraits::GetRaw(ns.left_);  }
        template <typename NodeState>
        static RawPtrType RLRawChild(NodeState& ns) { return PtrTraits::GetRaw(ns.right_); }

        static RawPtrType& LRMost(ContainerType& tree) { return tree.left_most_; }
        static RawPtrType& RLMost(ContainerType& tree) { return tree.right_most_; }
    };

    struct ReverseTraits {
        using Inverse = ForwardTraits;

        template <typename NodeState> static PtrType& LRChild(NodeState& ns) { return ns.right_; }
        template <typename NodeState> static PtrType& RLChild(NodeState& ns) { return ns.left_;  }

        template <typename NodeState>
        static RawPtrType LRRawChild(NodeState& ns) { return PtrTraits::GetRaw(ns.right_); }
        template <typename NodeState>
        static RawPtrType RLRawChild(NodeState& ns) { return PtrTraits::GetRaw(ns.left_);  }

        static RawPtrType& LRMost(ContainerType& tree) { return tree.right_most_; }
        static RawPtrType& RLMost(ContainerType& tree) { return tree.left_most_; }
    };

    // Trait classes used to define the bound condition for upper_bound and
    // lower_bound.
    struct UpperBoundTraits {
        static bool GoRight(const KeyType& key, const KeyType& node_key) {
            return KeyTraits::EqualTo(node_key, key) || KeyTraits::LessThan(node_key, key);
        }
    };

    struct LowerBoundTraits {
        static bool GoRight(const KeyType& key, const KeyType& node_key) {
            return KeyTraits::LessThan(node_key, key);
        }
    };

    // The shared implementation of the iterator
    template <class IterTraits>
    class iterator_impl {
    public:
        iterator_impl() { }
        iterator_impl(const iterator_impl& other) { node_ = other.node_; }

        iterator_impl& operator=(const iterator_impl& other) {
            node_ = other.node_;
            return *this;
        }

        bool IsValid() const { return PtrTraits::IsValid(node_); }
        bool operator==(const iterator_impl& other) const { return node_ == other.node_; }
        bool operator!=(const iterator_impl& other) const { return node_ != other.node_; }

        // Prefix
        iterator_impl& operator++() {
            if (IsValid())
                advance<ForwardTraits>();
            return *this;
        }

        iterator_impl& operator--() {
            // If this was a default constructed iterator, then it cannot
            // back up.
            if (node_ != nullptr) {
                // If we are at the end() of the tree, then recover the tree
                // pointer from the sentinel and back up to the right-most node.
                if (PtrTraits::IsSentinel(node_)) node_ = GetTree()->right_most_;
                else                              advance<ReverseTraits>();
            }

            return *this;
        }

        // Postfix
        iterator_impl operator++(int) {
            iterator_impl ret(*this);
            ++(*this);
            return ret;
        }

        iterator_impl operator--(int) {
            iterator_impl ret(*this);
            --(*this);
            return ret;
        }

        typename PtrTraits::PtrType CopyPointer() {
            return IsValid() ? PtrTraits::Copy(node_) : nullptr;
        }

        typename IterTraits::RefType operator*()     const { MX_DEBUG_ASSERT(node_); return *node_; }
        typename IterTraits::RawPtrType operator->() const { MX_DEBUG_ASSERT(node_); return node_; }

    private:
        friend ContainerType;

        iterator_impl(typename PtrTraits::RawPtrType node) : node_(node) { }

        ContainerType* GetTree() const {
            return reinterpret_cast<ContainerType*>(
                    reinterpret_cast<uintptr_t>(node_) & ~internal::kContainerSentinelBit);
        }

        template <typename LRTraits>
        void advance() {
            MX_DEBUG_ASSERT(PtrTraits::IsValid(node_));

            // Find the next node in the ordered sequecnce.
            // key.  This will be either...
            // 1) The RL-most child of our LR-hand sub-tree.
            // 2) Our first ancestor for which we are a LR-hand descendent.
            auto ns = &NodeTraits::node_state(*node_);
            auto rl_child = LRTraits::RLRawChild(*ns);
            if (rl_child != nullptr) {
                node_ = rl_child;

                // The RL-hand child of the RL-most node is terminated
                // using the sentinel value for this tree instead of nullptr.
                // Have we hit it?  If so, then we are done.
                if (PtrTraits::IsSentinel(node_))
                    return;

                // While we can go LR, do so.
                auto lr_child = LRTraits::LRRawChild(NodeTraits::node_state(*node_));
                while (lr_child != nullptr) {
                    MX_DEBUG_ASSERT(!PtrTraits::IsSentinel(lr_child));
                    node_    = lr_child;
                    lr_child = LRTraits::LRRawChild(NodeTraits::node_state(*node_));
                }
            } else {
                // Climb up the tree until we traverse a LR-hand link.  Because
                // of the sentinel termination, we should never attempt to climb
                // up past the root.
                bool done;
                auto ns = &NodeTraits::node_state(*node_);
                do {
                    MX_DEBUG_ASSERT(PtrTraits::IsValid(ns->parent_));

                    auto parent_ns = &NodeTraits::node_state(*ns->parent_);
                    done = (LRTraits::LRRawChild(*parent_ns) == node_);

                    MX_DEBUG_ASSERT(done || (LRTraits::RLRawChild(*parent_ns) == node_));

                    node_ = ns->parent_;
                    ns    = parent_ns;
                } while (!done);
            }
        }

        typename PtrTraits::RawPtrType node_ = nullptr;
    };  // class iterator_impl

    // The test framework's 'checker' class is our friend.
    friend CheckerType;

    // move semantics only
    DISALLOW_COPY_AND_ASSIGN_ALLOW_MOVE(WAVLTree);

    void internal_insert(PtrType&& ptr, RawPtrType* collision = nullptr) {
        MX_DEBUG_ASSERT(ptr != nullptr);

        auto& ns = NodeTraits::node_state(*ptr);
        MX_DEBUG_ASSERT(ns.IsValid() && !ns.InContainer());

        // The rank of an inserted node always starts at 0.
        ns.rank_ = 0;

        // If the tree is currently empty, then this is easy.
        if (root_ == nullptr) {
            ns.parent_ = sentinel();
            ns.left_   = PtrTraits::MakeSentinel(this);
            ns.right_  = PtrTraits::MakeSentinel(this);

            MX_DEBUG_ASSERT(PtrTraits::IsSentinel(left_most_) && PtrTraits::IsSentinel(right_most_));
            left_most_  = PtrTraits::GetRaw(ptr);
            right_most_ = PtrTraits::GetRaw(ptr);

            root_ = mxtl::move(ptr);

            ++count_;
            Observer::RecordInsert();
            return;
        }

        // Find the proper position for this node.
        auto key = KeyTraits::GetKey(*ptr);
        bool is_left_most  = true;
        bool is_right_most = true;
        RawPtrType parent  = PtrTraits::GetRaw(root_);
        PtrType* owner;

        while (true) {
            auto parent_key = KeyTraits::GetKey(*parent);

            // Looks like we collided with an object in the colleciton which has
            // the same key as the object being inserted.  This is only allowed
            // during an insert_or_find opertaion (collision is non-null).
            // Assert this in debug builds.  If this is an insert_or_find
            // operation, fill out the collision [out] parameter so the called
            // knows which object he/she collided with.  Either way, do not
            // actually insert the object.
            if (KeyTraits::EqualTo(key, parent_key)) {
                MX_DEBUG_ASSERT(collision);

                if (collision) {
                    MX_DEBUG_ASSERT(*collision == nullptr);
                    *collision = parent;
                }

                return;
            }

            auto& parent_ns = NodeTraits::node_state(*parent);

            // Decide which side of the current parent-under-consideration the
            // node to be inserted belongs on.  If we are going left, then we
            // are no longer right-most, and vice-versa.
            if (KeyTraits::LessThan(key, parent_key)) {
                owner = &parent_ns.left_;
                is_right_most = false;
            } else {
                owner = &parent_ns.right_;
                is_left_most = false;
            }

            // If we would have run out of valid pointers in the direction we
            // should be searching, then we are done.
            if (!PtrTraits::IsValid(*owner))
                break;

            // We belong on a side of the parent-under-consideration which
            // already has a child.  Move down to the child and consider it
            // instead.
            parent = PtrTraits::GetRaw(*owner);
        }

        // We know that we are not the root of the tree, therefore we cannot be
        // both left and right-most.
        MX_DEBUG_ASSERT(!is_left_most || !is_right_most);

        if (is_right_most) {
            MX_DEBUG_ASSERT(PtrTraits::IsSentinel(*owner));
            ns.right_   = PtrTraits::Take(*owner);
            right_most_ = PtrTraits::GetRaw(ptr);
        } else if (is_left_most) {
            MX_DEBUG_ASSERT(PtrTraits::IsSentinel(*owner));
            ns.left_   = PtrTraits::Take(*owner);
            left_most_ = PtrTraits::GetRaw(ptr);
        }

        MX_DEBUG_ASSERT(*owner == nullptr);
        ns.parent_ = parent;
        *owner = mxtl::move(ptr);

        ++count_;
        Observer::RecordInsert();

        // Finally, perform post-insert balance operations.
        BalancePostInsert(PtrTraits::GetRaw(*owner));
    }

    PtrType internal_erase(RawPtrType ptr) {
        if (!PtrTraits::IsValid(ptr))
            return PtrType(nullptr);

        auto& ns = NodeTraits::node_state(*ptr);
        PtrType* owner;

        // If the target node is the root of the tree, then its parent will be
        // sentinel value and the owning pointer will be the root pointer.
        MX_DEBUG_ASSERT(ns.parent_ != nullptr);
        if (PtrTraits::IsSentinel(ns.parent_)) {
            owner = &root_;
        } else {
            // Determine if we are our parent's left or right child so we can
            // select the proper owning pointer.
            auto& parent_ns = NodeTraits::node_state(*ns.parent_);

            owner = PtrTraits::GetRaw(parent_ns.left_) == ptr
                  ? &parent_ns.left_
                  : &parent_ns.right_;

        }
        MX_DEBUG_ASSERT(PtrTraits::GetRaw(*owner) == ptr);

        // If the node we want to remove has two children, swap it with the
        // left-most node of the right-hand sub-tree before proceeding.  This
        // will guarantee that we are operating in a 0 or 1 child case.
        if (PtrTraits::IsValid(ns.left_) && PtrTraits::IsValid(ns.right_)) {
            PtrType*   new_owner = &ns.right_;
            auto       new_ns    = &NodeTraits::node_state(*ns.right_);

            while (new_ns->left_ != nullptr) {
                MX_DEBUG_ASSERT(!PtrTraits::IsSentinel(new_ns->left_));
                new_owner = &new_ns->left_;
                new_ns = &NodeTraits::node_state(*new_ns->left_);
            }

            owner = SwapWithRightDescendant(*owner, *new_owner);
            MX_DEBUG_ASSERT(PtrTraits::GetRaw(*owner) == ptr);
        }

        // Now that we know our relationship with our parent, go ahead and start
        // the process of removing the node.  Keep track of the target node's
        // parent, whether it was it's parent's left or right child, and whether
        // it was a 1-child or a 2-child.  We will need this info when it comes
        // time to rebalance.
        RawPtrType parent = ns.parent_;
        bool was_one_child, was_left_child;

        MX_DEBUG_ASSERT(parent != nullptr);
        if (!PtrTraits::IsSentinel(parent)) {
            auto& parent_ns = NodeTraits::node_state(*parent);

            was_one_child  = ns.rank_parity() != parent_ns.rank_parity();
            was_left_child = &parent_ns.left_ == owner;
        } else {
            was_one_child  = false;
            was_left_child = false;
        }

        // Now, detach the node from its owner (the pointer which points to it).
        PtrType removed = PtrTraits::Take(*owner);

        // We know that we have at most one child.  If we do have a child,
        // promote it to our position.  While we are handling the cases,
        // maintain the LR-most bookkeeping.  Consider the 1 and 0 child cases
        // separately.
        //
        // 1-child case:
        // We are promoting the LR-child.  If we were RL-most, then our RL-child
        // is the sentinel value.  We need to transfer this sentinel value to
        // the RL-most node in our LR-subtree.  We also need to update the
        // parent pointer of our LR-child to point to the removed node's parent.
        //
        // 0-child case:
        // We are not promoting any node, but we may have been either the
        // left-most or not, and either the right-most node, or not.
        //
        // ++ If the target is both the left and right-most node in the tree, it
        //    is a leaf, then the target *must* be the final node in the tree.  The
        //    tree's left and right-most need to be updated to be the sentinel
        //    value, and the sentinels need to be detached from the target node's
        //    state structure.
        // ++ If the target is the LR-most node in the tree and a leaf, then its
        //    parent is now the LR-most node in the tree.  We need to transfer
        //    the sentinel from the target's LR-child to the pointer which used
        //    to point to it, and update the LR-most bookkeeping in the tree to
        //    point to the target's parent. The target node's RL-child should
        //    already be nullptr.
        // ++ If the target neither the left nor the right most node in the
        //    tree, then nothing special needs to happen with regard to the
        //    left/right-most bookkeeping.
        RawPtrType target = PtrTraits::GetRaw(removed);
        if      (PtrTraits::IsValid(ns.left_))  PromoteLRChild<ForwardTraits>(*owner, target);
        else if (PtrTraits::IsValid(ns.right_)) PromoteLRChild<ReverseTraits>(*owner, target);
        else {
            // The target's LR-child is the sentinel if and only if the target
            // is the LR-most node in the tree.
            MX_DEBUG_ASSERT(PtrTraits::IsSentinel(ns.left_)  == (left_most_  == target));
            MX_DEBUG_ASSERT(PtrTraits::IsSentinel(ns.right_) == (right_most_ == target));

            if (PtrTraits::IsSentinel(ns.left_)) {
                if (PtrTraits::IsSentinel(ns.right_)) {
                    // Target is both left and right most.
                    MX_DEBUG_ASSERT(count_ == 1);
                    MX_DEBUG_ASSERT(PtrTraits::IsSentinel(ns.parent_));
                    left_most_  = sentinel();
                    right_most_ = sentinel();
                    PtrTraits::DetachSentinel(ns.left_);
                    PtrTraits::DetachSentinel(ns.right_);
                } else {
                    // Target is just left most.
                    MX_DEBUG_ASSERT(PtrTraits::IsValid(ns.parent_));
                    left_most_ = ns.parent_;
                    *owner = PtrTraits::Take(ns.left_);
                }
            } else if (PtrTraits::IsSentinel(ns.right_)) {
                    // Target is just right most.
                    MX_DEBUG_ASSERT(PtrTraits::IsValid(ns.parent_));
                    right_most_ = ns.parent_;
                    *owner = PtrTraits::Take(ns.right_);
            }

            // Disconnect the target's parent pointer and we should be done.
            ns.parent_ = nullptr;
        }

        // At this point in time, the target node should have been completely
        // removed from the tree.  Its internal state should be valid, and
        // indicate that it is not in the container.
        MX_DEBUG_ASSERT(ns.IsValid() && !ns.InContainer());

        // Update the count bookkeeping.
        --count_;
        Observer::RecordErase();

        // Time to rebalance.  We know that we don't need to rebalance if we
        // just removed the root (IOW - its parent was the sentinel value).
        if (!PtrTraits::IsSentinel(parent)) {
            if (was_one_child) {
                // If the node we removed was a 1-child, then we may have just
                // turned its parent into a 2,2 leaf node.  If so, we have a
                // leaf node with non-zero rank and need to fix the problem.
                BalancePostErase_Fix22Leaf(parent);
            } else {
                // If the node we removed was a 2-child, the we may have just
                // created a 3 child which we need to fix.  The balance routine
                // will handle checking for us, but it will need to know whether
                // the node we removed was its parent's left or right child in
                // order to un-ambiguously perform the check.
                if (was_left_child) BalancePostErase_FixLR3Child<ForwardTraits>(parent);
                else                BalancePostErase_FixLR3Child<ReverseTraits>(parent);
            }
        }

        // Release the pointer to the node we just removed back to the caller.
        return removed;
    }

    template <typename BoundTraits>
    const_iterator internal_upper_lower_bound(const KeyType& key) const {
        RawPtrType node  = PtrTraits::GetRaw(root_);
        RawPtrType found = sentinel();

        while (PtrTraits::IsValid(node)) {
            auto node_key = KeyTraits::GetKey(*node);

            if (BoundTraits::GoRight(key, node_key)) {
                // If we need to look for a larger node value (key > node_key in
                // the case of lower_bound, key >= node_key in the case of
                // upper_bound), then go right.  If we cannot go right, then
                // there is other element in this container whose key satisfies
                // the bound condition.  Break out of the loop and return the
                // best node we have found so far.
                auto& ns = NodeTraits::node_state(*node);

                if (ns.right_ == nullptr)
                    break;

                node = PtrTraits::GetRaw(ns.right_);
            } else {
                // If this node's key must be greater than or equal to the
                // user's key.  This node is now our candidate for our found
                // node.
                found = node;

                // If this node has a left-hand child, it is possible that there
                // is a better bound somewhere underneath it.  Set the node
                // pointer to the root of the left hand sub-tree and keep
                // looking.
                node = PtrTraits::GetRaw(NodeTraits::node_state(*node).left_);
            }
        }

        return const_iterator(found);
    }

    constexpr RawPtrType sentinel() const {
        return reinterpret_cast<RawPtrType>(
                reinterpret_cast<uintptr_t>(this) | internal::kContainerSentinelBit);
    }

    template <typename T>
    static void pod_swap(T& first, T& second) {
        T tmp  = first;
        first  = second;
        second = tmp;
    }

    // Swaps the positions of two nodes, one of which is guaranteed to be a
    // right-hand descendant of the other.
    //
    // @note Node #1 is the ancestor node while node #2 is the descendant node.
    //
    // @param  ptr_ref1 A reference to the pointer which points to node #1.
    // @param  ptr_ref2 A reference to the pointer which points to node #2.
    // @return The new pointer to the pointer which points to node #1.
    //
    PtrType* SwapWithRightDescendant(PtrType& ptr_ref1, PtrType& ptr_ref2) {
        RawPtrType node1 = PtrTraits::GetRaw(ptr_ref1);
        RawPtrType node2 = PtrTraits::GetRaw(ptr_ref2);

        auto& ns1 = NodeTraits::node_state(*node1);
        auto& ns2 = NodeTraits::node_state(*node2);

        auto ns1_lp = PtrTraits::IsValid(ns1.left_)
                    ? &NodeTraits::node_state(*ns1.left_).parent_
                    : nullptr;
        auto ns2_lp = PtrTraits::IsValid(ns2.left_)
                    ? &NodeTraits::node_state(*ns2.left_).parent_
                    : nullptr;
        auto ns2_rp = PtrTraits::IsValid(ns2.right_)
                    ? &NodeTraits::node_state(*ns2.right_).parent_
                    : nullptr;

        // node 2 is a right-hand descendant of node 1, so node 1's right hand
        // pointer must be valid.
        MX_DEBUG_ASSERT(PtrTraits::IsValid(ns1.right_));
        auto ns1_rp = &NodeTraits::node_state(*ns1.right_).parent_;

        // Start by updating the LR-most bookkeeping.  Node 1 cannot be the
        // right-most node, and node 2 cannot be the left most node (because we
        // know that node 2 is to the right of node 1).
        //
        // If node 1 is currently the left-most, then node 2 will be when we are
        // done.  Likewise, if node 2 is currently the right-most, then node 1
        // will be the right-most when we are done.
        if (node1 == left_most_)  left_most_  = node2;
        if (node2 == right_most_) right_most_ = node1;

        // Next, swap the core state of node 1 and node 2.
        pod_swap(ns1.parent_, ns2.parent_);
        PtrTraits::Swap(ns1.left_,  ns2.left_);
        PtrTraits::Swap(ns1.right_, ns2.right_);
        pod_swap(ns1.rank_, ns2.rank_);

        // At this point, there are two scenarios.
        //
        // Case #1
        // Node 2 was an indirect descendant of node 1.  In this case, all we
        // need to do is swap the ptr_ref[12] pointers and fix-up the various
        // children's parent pointers (when they exist).
        //
        // Case #2
        // Node 2 was the direct descendant of node 1; in this case the right
        // hand child.  In this case, we know 2 things...
        //
        // 1) node1.right is the same pointer as ptr_ref2
        // 2) node2.parent is the same pointer as node 1's right-child's parent.
        //
        // Because of this (and because of the swapping of core state prior to
        // this), we know...
        //
        // 1) node1.parent currently points to node 1, but should point to node 2.
        // 2) node2.right currently points to node 2, but should point to node 1.
        // 3) ptr_ref1 still points to node 1, but should point to node 2.
        // 4) node2.parent (aka; ns1_rp) currently points to node 1's old
        //    parent (which is correct).
        //
        // We fix issues 1-3 by swapping ptr_ref1 and node2.right, and by
        // directly setting setting node1.parent to node2.
        //
        // Finally, we need to return the new pointer to the pointer which
        // points to node 1.  In case #1, this is just ptr_ref2.  In case #2,
        // however, this has become node 2's right hand pointer.
        //
        // Perform the common child fixup first, then deal with the special
        // cases.
        if (ns1_lp) *ns1_lp = node2;
        if (ns2_lp) *ns2_lp = node1;
        if (ns2_rp) *ns2_rp = node1;

        if (&ptr_ref2 != &ns1.right_) {
            // Case #1.
            PtrTraits::Swap(ptr_ref1, ptr_ref2);
            *ns1_rp = node2;
            return &ptr_ref2;
        } else {
            MX_DEBUG_ASSERT(ns1.parent_ == node1);
            MX_DEBUG_ASSERT(PtrTraits::GetRaw(ns2.right_) == node2);
            PtrTraits::Swap(ptr_ref1, ns2.right_);
            ns1.parent_ = node2;
            return &ns2.right_;
        }
    }

    // Promote the LR-child of a node to be removed into the node's position.
    // Update the LR-node bookkeeping in the process.  The LRTraits will
    // determine if we are promoting the left or right child (the forward
    // operation promotes left).
    //
    // Requirements:
    // ++ The node being removed *must* have an LR-child.
    // ++ The node being removed *must not* have an RL-child.
    // ++ The owner reference *must* have already been disconnected from the
    //    node to be removed.
    //
    // @param owner A reference to the pointer which points to the node to be
    // removed.
    // @param node A pointer to the node to be removed.
    //
    template <typename LRTraits>
    void PromoteLRChild(PtrType& owner, RawPtrType node) {
        MX_DEBUG_ASSERT(owner == nullptr);
        MX_DEBUG_ASSERT(node != nullptr);

        auto& ns = NodeTraits::node_state(*node);
        PtrType& lr_child = LRTraits::LRChild(ns);
        PtrType& rl_child = LRTraits::RLChild(ns);

        MX_DEBUG_ASSERT(PtrTraits::IsValid(lr_child) && !PtrTraits::IsValid(rl_child));

        // Promote by transferring the LR-Child pointer to the owner pointer and
        // fixing up the LR-Child's parent pointer be the current parent of the
        // node to be removed.
        owner = PtrTraits::Take(lr_child);  // owner now points to the promoted node.
        NodeTraits::node_state(*owner).parent_ = ns.parent_;

        // The removed node is the RL-most node if (and only if) its RL-child
        // was the sentinel value.
        RawPtrType& rl_most = LRTraits::RLMost(*this);
        MX_DEBUG_ASSERT((rl_most == node) == (PtrTraits::IsSentinel(rl_child)));
        if (PtrTraits::IsSentinel(rl_child)) {
            // The target node was the RL-most.  Find the new RL-most node. It will
            // be the RL-most node in the LR-subtree of the target node.  Once
            // found, update the RL-child of the new RL-most node to be the
            // sentinel value.
            RawPtrType replacement = PtrTraits::GetRaw(owner);
            PtrType*   next_rl_child;

            while (true) {
                auto& replacement_ns = NodeTraits::node_state(*replacement);
                next_rl_child = &LRTraits::RLChild(replacement_ns);

                MX_DEBUG_ASSERT(!PtrTraits::IsSentinel(*next_rl_child));
                if (*next_rl_child == nullptr)
                    break;

                replacement = PtrTraits::GetRaw(*next_rl_child);
            }

            // Update the bookkeeping, detaching the sentinel value from the
            // RL-child of the target node in the process.
            rl_most = replacement;
            *next_rl_child = PtrTraits::Take(rl_child);
        }

        // Unlink the parent pointer for the target node and we should be done.
        // The left and right children of the target node should already be
        // nullptr by now.
        ns.parent_ = nullptr;
        MX_DEBUG_ASSERT(ns.left_  == nullptr);
        MX_DEBUG_ASSERT(ns.right_ == nullptr);
    }

    // After we have swapped contents with another tree, we need to fix up the
    // sentinel values so that they refer to the proper tree.  Otherwise tree
    // A's sentinels will point at tree B's, and vice-versa.
    void FixSentinelsAfterSwap() {
        if (root_) {
            MX_DEBUG_ASSERT(!PtrTraits::IsSentinel(root_));
            MX_DEBUG_ASSERT(left_most_ &&  !PtrTraits::IsSentinel(left_most_));
            MX_DEBUG_ASSERT(right_most_ && !PtrTraits::IsSentinel(right_most_));

            auto& root_ns       = NodeTraits::node_state(*root_);
            auto& left_most_ns  = NodeTraits::node_state(*left_most_);
            auto& right_most_ns = NodeTraits::node_state(*right_most_);

            MX_DEBUG_ASSERT(PtrTraits::IsSentinel(left_most_ns.left_));
            MX_DEBUG_ASSERT(PtrTraits::IsSentinel(right_most_ns.right_));

            PtrTraits::DetachSentinel(left_most_ns.left_);
            PtrTraits::DetachSentinel(right_most_ns.right_);
            root_ns.parent_      = sentinel();
            left_most_ns.left_   = PtrTraits::MakeSentinel(this);
            right_most_ns.right_ = PtrTraits::MakeSentinel(this);
        } else {
            MX_DEBUG_ASSERT(PtrTraits::IsSentinel(left_most_));
            MX_DEBUG_ASSERT(PtrTraits::IsSentinel(right_most_));
            left_most_  = sentinel();
            right_most_ = sentinel();
        }
    }

    // GetLinkPtrToNode.
    //
    // Obtain a reference to the pointer which points to node.  The will either be
    // a reference to the node's parent's left child, right child, or the root
    // node of the tree if the child has no parent.
    PtrType& GetLinkPtrToNode(RawPtrType node) {
        MX_DEBUG_ASSERT(PtrTraits::IsValid(node));

        auto& ns = NodeTraits::node_state(*node);
        if (PtrTraits::IsSentinel(ns.parent_)) {
            MX_DEBUG_ASSERT(ns.parent_ == sentinel());
            MX_DEBUG_ASSERT(PtrTraits::GetRaw(root_) == node);
            return root_;
        }

        MX_DEBUG_ASSERT(ns.parent_ != nullptr);
        auto& parent_ns = NodeTraits::node_state(*ns.parent_);
        if (PtrTraits::GetRaw(parent_ns.left_) == node)
            return parent_ns.left_;

        MX_DEBUG_ASSERT(PtrTraits::GetRaw(parent_ns.right_) == node);
        return parent_ns.right_;
    }

    // RotateLR<LRTraits>
    //
    // Perform a Rotate-LR operation at 'node'.
    //
    // Let...
    // X = node
    // Y = LR-Child of X (if any)
    // Z = X's parent.
    //
    // A Rotate-LR operation at 'node' is defined as follows.
    // 1) Z becomes the LR-Child of X
    // 2) Y becomes the RL-Child of Z
    // 3) X takes Z's position in the tree
    //
    // If [XYZ]_link is the pointer which points to [XYZ], then we can describe
    // the operation as the following permutation of the links.
    //
    //   link  | before | swap1 | swap2 |
    // --------+--------+-------+-------+
    //  X_link |   X    |   Y   |   Y   +
    //  Y_link |   Y    |   X   |   Z   +
    //  Z_link |   Z    |   Z   |   X   +
    //
    // Note that link's are bi-directional.  We need to update the parent
    // pointers as well.  Let G be Z's parent at the start of the operation.
    // The permutation should be.
    //
    //   node  | P(n) before | P(n) after |
    // --------+-------------+------------+
    //    X    |      Z      |     G      |
    //    Y    |      X      |     Z      |
    //    Z    |      G      |     X      |
    //
    // We should not need to worry about the LR-most-ness of any of the nodes.
    // Reasoning is as follows...
    //
    // 1) Z might be the LR-most node in the tree, but only if it has no
    //    LR-Child.  It cannot be the RL-most node in the tree, because we know
    //    that it has an RL-child (X).  When we rotate, Z moves to the LR.  So
    //    if it or one of its LR-children was LR-most, they remain that way.
    // 2) X might be the RL-most node in the tree, but only if it has no
    //    RL-child.  It cannot be the LR-most node in the tree because it is the
    //    RL-child of Z.  When we rotate, X moves up in the tree following Z's
    //    RL-child link.  This means that if X was on the RL-edge of the tree
    //    (implying that it or one of its children was RL-most), it remains
    //    there, preserving the RL-most-ness of it or its children.
    // 3) Before the rotation, Y cannot be either the RL-most node in the tree
    //    (it is X's LR-child), or the LR-most node in the tree (it's parent, X,
    //    is Z's RL-child).  After the rotation, Y still cannot be either
    //    LR-most (it is now Z's RL-child) or RL-most (it's parent, Z, is X's
    //    LR-child).
    template <typename LRTraits>
    void RotateLR(RawPtrType node, RawPtrType parent) {
        MX_DEBUG_ASSERT(PtrTraits::IsValid(node));     // Node must be valid
        MX_DEBUG_ASSERT(PtrTraits::IsValid(parent));   // Node must have a parent

        // Aliases, just to make the code below match the notation used above.
        RawPtrType X = node;
        RawPtrType Z = parent;

        auto& X_ns = NodeTraits::node_state(*X);
        auto& Z_ns = NodeTraits::node_state(*Z);

        // X must be the RL-child of Z.
        MX_DEBUG_ASSERT(LRTraits::RLRawChild(Z_ns) == X);

        PtrType& X_link = LRTraits::RLChild(Z_ns);
        PtrType& Y_link = LRTraits::LRChild(X_ns);
        PtrType& Z_link = GetLinkPtrToNode(Z);

        RawPtrType G = Z_ns.parent_;
        RawPtrType Y = PtrTraits::GetRaw(Y_link);

        // The pointer to Y cannot be a sentinel, because that would imply that
        // X was LR-most.
        MX_DEBUG_ASSERT(!PtrTraits::IsSentinel(Y));

        // Permute the downstream links.
        PtrTraits::Swap(X_link, Y_link);
        PtrTraits::Swap(Y_link, Z_link);

        // Update the parent pointers (note that Y may not exist).
        X_ns.parent_ = G;
        Z_ns.parent_ = X;
        if (Y)
            NodeTraits::node_state(*Y).parent_ = Z;
    }

    // PostInsertFixupLR<LRTraits>
    //
    // Called after the promotion stage of an insert operation, when node's
    // parent has become 0,2 node and the rank rule needs to be restored.
    //
    // If node is the LR-child of parent, define...
    // X = node
    // Y = RL-child of X
    // Z = parent
    //
    // There are two possibilities of what to do next.
    //
    // if (Y is null) or (Y is a 2-child)
    // then...
    // 1) rotate-RL about X
    // 2) demote Z
    //
    // OR
    //
    // if (Y is a 1-child)
    // then...
    // 1) rotate-LR about Y
    // 2) rotate-RL about Y
    // 3) promote Y
    // 4) demote X
    // 5) demote Z
    template <typename LRTraits>
    void PostInsertFixupLR(RawPtrType node, RawPtrType parent) {
        using RLTraits = typename LRTraits::Inverse;

        MX_DEBUG_ASSERT(PtrTraits::IsValid(node));
        MX_DEBUG_ASSERT(PtrTraits::IsValid(parent));

        auto& node_ns   = NodeTraits::node_state(*node);
        auto& parent_ns = NodeTraits::node_state(*parent);

        MX_DEBUG_ASSERT(LRTraits::LRRawChild(parent_ns) == node);

        RawPtrType rl_child = LRTraits::RLRawChild(node_ns);
        auto rl_child_ns    = PtrTraits::IsValid(rl_child)
                            ? &NodeTraits::node_state(*rl_child)
                            : nullptr;
        if (!rl_child_ns || (rl_child_ns->rank_parity() == node_ns.rank_parity())) {
            // Case #1; single rotation
            //
            // Start by performing a Rotate-RL at node
            RotateLR<RLTraits>(node, parent);
            Observer::RecordInsertRotation();

            // Now demote node's old parent (now its LR-child)
            parent_ns.demote_rank();
        } else {
            // Case #2; double rotation.
            //
            // Start by performing a Rotate-LR at rl_child.  Afterwards,
            // rl_child will be LR-child of parent (and node is now its
            // LR-child).  Rotate-RL at rl_child.
            RotateLR<LRTraits>(rl_child, node);
            RotateLR<RLTraits>(rl_child, parent);
            Observer::RecordInsertDoubleRotation();

            // 1 promotion and 2 demotions.
            rl_child_ns->promote_rank();
            node_ns.demote_rank();
            parent_ns.demote_rank();
        }
    }

    // BalancePostInsert
    //
    // Execute the bottom-up post-insert rebalancing algorithm.
    //
    // @param node A pointer to the node which was just inserted.
    //
    void BalancePostInsert(RawPtrType node) {
        // We do not balance the tree after inserting the first (root)
        // node, so we should be able to assert that we have a valid parent.
        auto node_ns = &NodeTraits::node_state(*node);
        MX_DEBUG_ASSERT(PtrTraits::IsValid(node_ns->parent_));

        // If we have a sibling, then our parent just went from being a 1,2
        // unary node into a 1,1 binary node and no action needs to be taken.
        RawPtrType parent = node_ns->parent_;
        auto parent_ns = &NodeTraits::node_state(*parent);
        if (PtrTraits::IsValid(parent_ns->left_) && PtrTraits::IsValid(parent_ns->right_))
            return;

        // We have no sibling, therefor we just changed our parent from a 1,1
        // leaf node into a 0,1 unary node.  The WAVL rank rule states that all
        // rank differences are 1 or 2; 0 is not allowed.  Rebalance the tree in
        // order to restore the rule.
        //
        // Start with the promotions.  Pseudo code is...
        //
        // while (IsValid(parent) && parent is a 0,1 node)
        //    promote parent
        //    climb up the tree
        bool node_parity;
        bool parent_parity;
        bool sibling_parity;
        bool is_left_child;
        do {
            // Promote
            parent_ns->promote_rank();
            Observer::RecordInsertPromote();

            // Climb
            node    = parent;
            node_ns = &NodeTraits::node_state(*node);
            parent  = node_ns->parent_;

            // If we have run out of room to climb, then we must be done.
            if (!PtrTraits::IsValid(parent))
                return;

            // We have a parent.  Determine the relationship between our rank
            // parity,  our parent's rank parity, and our sibling's rank parity
            // (if any).  Note; null children have a rank of -1, therefor the
            // rank parity of a sibling is always odd (1).  In the process, make
            // a note of whether we are our parent's left or right child.
            parent_ns = &NodeTraits::node_state(*parent);
            is_left_child = (PtrTraits::GetRaw(parent_ns->left_) == node);
            if (is_left_child) {
                sibling_parity = PtrTraits::IsValid(parent_ns->right_)
                               ? NodeTraits::node_state(*parent_ns->right_).rank_parity()
                               : true;
            } else {
                MX_DEBUG_ASSERT(PtrTraits::GetRaw(parent_ns->right_) == node);
                sibling_parity = PtrTraits::IsValid(parent_ns->left_)
                               ? NodeTraits::node_state(*parent_ns->left_).rank_parity()
                               : true;
            }

            node_parity   = node_ns->rank_parity();
            parent_parity = parent_ns->rank_parity();

            // We need to keep promoting and climbing if we just turned our new
            // parent into a 0,1 node.  Let N, P and S denote the current node,
            // parent and sibling parities.  Working out the truth tables, our
            // parent is now a 0,1 node iff (!N * !P * S) + (N * P * !S)
        } while ((!node_parity && !parent_parity &&  sibling_parity) ||
                 ( node_parity &&  parent_parity && !sibling_parity));

        // OK.  At this point, we know that we have a parent, and our parent is
        // not a 0,1 node.  Either our rank rule has been restored and we are
        // done, or our parent is 0,2 and we need to perform either one or two
        // rotations.  Start by checking to see if our parent is 0,2.  Using the
        // notation from above...
        //
        // ++ our parent is a 0,2 node iff (!N * !P * !S) + (N * P * S).
        // ++ which implies that we are finished iff (N != P) + (N != S)
        //
        if ((node_parity != parent_parity) || (node_parity != sibling_parity))
            return;

        // Looks like our parent is a 0,2 node.  Perform the post-insert fixup
        // operations, forward if we are our parent's left child, reverse if we
        // are our parent's right child.
        if (is_left_child) PostInsertFixupLR<ForwardTraits>(node, parent);
        else               PostInsertFixupLR<ReverseTraits>(node, parent);
    }

    // BalancePostErase_Fix22Leaf
    //
    // Called after an erase operation which erased the 1-child of a node.
    // Checks to see if the node has become a 2,2 leaf node and takes
    // appropriate action to restore the rank rule if needed.
    void BalancePostErase_Fix22Leaf(RawPtrType node) {
        MX_DEBUG_ASSERT(PtrTraits::IsValid(node));

        // If we just turned node into a 2,2 leaf, it will have no children and
        // odd rank-parity.  If it has even parity, or any children at all,
        // there is nothing we need to do.
        auto& ns = NodeTraits::node_state(*node);
        if (!ns.rank_parity() ||
            PtrTraits::IsValid(ns.left_) ||
            PtrTraits::IsValid(ns.right_))
            return;

        // Demote the node turning it into a 1,1 leaf.
        ns.demote_rank();
        Observer::RecordEraseDemote();

        // By demoting this node, we may have just created a 3-child.  Find its
        // parent, figure out if it was the left or right child, then use the
        // FixLR3Child method to check for the 3-child case and deal with it if
        // we need to.  If this node had no parent, then we know that we are
        // finished.
        MX_DEBUG_ASSERT(ns.parent_ != nullptr);
        if (PtrTraits::IsSentinel(ns.parent_))
            return;

        auto& parent_ns     = NodeTraits::node_state(*ns.parent_);
        bool  is_left_child = PtrTraits::GetRaw(parent_ns.left_) == node;
        MX_DEBUG_ASSERT(is_left_child || (PtrTraits::GetRaw(parent_ns.right_) == node));

        if (is_left_child) BalancePostErase_FixLR3Child<ForwardTraits>(ns.parent_);
        else               BalancePostErase_FixLR3Child<ReverseTraits>(ns.parent_);
    }

    // BalancePostErase_FixLR3Child<LRTraits>
    //
    // Called during post-erase rebalancing when it is possible that a 3-child
    // may have been created.  There are 3 ways that this may have happened.
    //
    // 1) A node's 2-child leaf node was erased making the link to null a
    //    3-child link.
    // 2) A node's 2-child unary node was erased making the promoted node
    //    a 3-child.
    // 3) During rebalancing fix-up of a 2,2 leaf, the 2,2 leaf node is a
    //    2-child.  Demoting it to turn it into a 1,1 leaf makes it a 3-child.
    //
    // Note: this method is templated using LRTraits because the method needs to
    // know which link may have become a 3-child based on prior circumstances.
    // Without this knowledge, it is not possible to detect a 3 child using only
    // rank parity.  For this method, the LR-Child of 'node' is known to now be
    // either a 2-child or a 3-child, it cannot be a 1-child.
    template <typename LRTraits>
    void BalancePostErase_FixLR3Child(RawPtrType node) {
        using RLTraits = typename LRTraits::Inverse;
        MX_DEBUG_ASSERT(PtrTraits::IsValid(node));

        // Throughout this method, we will use the following notation.
        //
        // Let...
        // Z = The node with the (potential) 3-child.
        // X = The (potential) 3-child.
        // Y = X's sibling. (if any)
        //
        RawPtrType Z    = node;
        auto       Z_ns = &NodeTraits::node_state(*Z);
        RawPtrType X    = LRTraits::LRRawChild(*Z_ns);

        // Check to see if X is a 3-child of Z.  We know it is if...
        // 1) X exists and Z has odd rank parity
        // 2) X does not exist and Z has even rank parity
        if (PtrTraits::IsValid(X) != Z_ns->rank_parity())
            return;

        // Phase 1, demotions.
        //
        // While X is a 3-child and Y is a 2-child, or a 2,2 node
        //    Demote Y if it is is a 2,2 node
        //    Demote Z
        //    Climb (set Z = Z-parent, update definitions of X and Y)
        //
        bool X_is_LR_child = true;
        RawPtrType Y = LRTraits::RLRawChild(*Z_ns);
        while (true) {
            // We know that X is a 3 child, Determine the status of Y.  We know
            // that whenever X is a 3-child that Y must exist because...
            //
            // 1) Z's rank is at least 2. In the case that X does not exist, X's rank
            //    is -1 so Z's rank is (-1 + 3) = 2.  If X does exist, Z's rank
            //    is even larger.
            // 2) The rank rule for the Y,Z relationship should currently hold,
            //    meaning that the rank difference between Y and Z is either 1 or 2,
            //    therefor Y's rank is at least 0.
            // 3) Because Y has non-negative rank, it must exist.
            MX_DEBUG_ASSERT(PtrTraits::IsValid(Y));

            auto& Y_ns = NodeTraits::node_state(*Y);
            bool  Y_is_2_child = (Y_ns.rank_parity() == Z_ns->rank_parity());

            // Our next steps are already determined and don't involve Y if
            // Y is currently a 2 child.  Don't bother to compute
            // Y_is_22_node if we already know that Y is a 2-child.
            if (!Y_is_2_child) {
                bool Y_is_22_node;
                if (Y_ns.rank_parity()) {
                    // If Y has odd rank parity, it is a 2,2 node if both its
                    // children have odd parity, meaning each child either does
                    // not exist, or exists and has odd parity..
                    Y_is_22_node = ((!PtrTraits::IsValid(Y_ns.left_) ||
                                     NodeTraits::node_state(*Y_ns.left_).rank_parity()) &&
                                    (!PtrTraits::IsValid(Y_ns.right_) ||
                                     NodeTraits::node_state(*Y_ns.right_).rank_parity()));
                } else {
                    // If Y has even rank parity, it can only be a 2,2 node if it is
                    // a binary node and both of its children have even parity.
                    Y_is_22_node = PtrTraits::IsValid(Y_ns.left_) &&
                                   PtrTraits::IsValid(Y_ns.right_) &&
                                   !NodeTraits::node_state(*Y_ns.left_).rank_parity() &&
                                   !NodeTraits::node_state(*Y_ns.right_).rank_parity();
                }

                // If Y is neither a 2-child or a 2,2 node, then we are done
                // with phase 1.  X is still a 3-child, but it will take one or
                // more rotations to fix the problem.
                if (!Y_is_22_node)
                    break;
            }

            // Demote Z.  If Y was a 1-child, demote Y as well.
            Z_ns->demote_rank();
            Observer::RecordEraseDemote();

            if (!Y_is_2_child) {
                Y_ns.demote_rank();
                Observer::RecordEraseDemote();
            }

            // Climb.  If we cannot climb because we have no parent, or we can
            // climb and the new X is no longer a 3-child, then we are done
            // with the rebalance operation.
            if (!PtrTraits::IsValid(Z_ns->parent_))
                return;

            bool X_rank_parity = Z_ns->rank_parity();
            X    = Z;
            Z    = Z_ns->parent_;
            Z_ns = &NodeTraits::node_state(*Z);
            if (Z_ns->rank_parity() == X_rank_parity)
                return;

            X_is_LR_child = (LRTraits::LRRawChild(*Z_ns) == X);
            Y = X_is_LR_child
              ? LRTraits::RLRawChild(*Z_ns)
              : LRTraits::LRRawChild(*Z_ns);
        }

        // Phase 2, rotations
        //
        // At this point, we know...
        //
        // 1) Z exists
        // 2) X is a 3-child of Z (but may not exist).
        // 3) Y exists (because X is a 3-child of Z, see above)
        // 4) Y is not a 2-child of Z (so, Z is a 1,3 node)
        // 5) Y is not a 2,2 node.
        //
        // We will need to perform either 1 or 2 rotations to fix this problem.
        // Which directions we rotate will now depend on whether or not X is a
        // left or right child of Z.  Invoke the post-erase rotation method
        // using the traits which will normalize the notation so that X is an
        // LR-child of Z.
        if (X_is_LR_child) BalancePostErase_DoRotations<LRTraits>(Y, Z);
        else               BalancePostErase_DoRotations<RLTraits>(Y, Z);
    }

    // BalancePostErase_DoRotations<LRTraits>
    //
    // Refer to the notes at the end of BalancePostErase_FixLR3Child<LRTraits>
    // for a description of the current situation.  In addition to the
    // assertions there, we also now know that X is the LR-Child of Z (and
    // therefor Y is the RL-child of Z) (note; X is only indirectly involved
    // here, so not passed as an argument).
    template <typename LRTraits>
    void BalancePostErase_DoRotations(RawPtrType Y, RawPtrType Z) {
        using RLTraits = typename LRTraits::Inverse;
        MX_DEBUG_ASSERT(PtrTraits::IsValid(Y));
        MX_DEBUG_ASSERT(PtrTraits::IsValid(Z));

        auto& Y_ns = NodeTraits::node_state(*Y);
        auto& Z_ns = NodeTraits::node_state(*Z);

        // Let...
        // V = the LR-child of Y
        // W = the RL-child of Y
        //
        // Perform rotations to fix the fact that X is a 3-child of Z.  We will
        // need to do 1 of 2 things depending on whether W is a 1-child or
        // 2-child of Y.
        //
        RawPtrType W = LRTraits::RLRawChild(Y_ns);
        bool W_rank_parity = PtrTraits::IsValid(W)
                           ? NodeTraits::node_state(*W).rank_parity()
                           : true;
        if (Y_ns.rank_parity() != W_rank_parity) {
            // Case #1: W is a 1-child of Y
            //
            // 1)  Rotate-LR at Y
            // 2)  Promote Y
            // 3a) If Z is a leaf, demote it twice.
            // 3b) else demote Z once.
            RotateLR<LRTraits>(Y, Z);
            Observer::RecordEraseRotation();

            Y_ns.promote_rank();

            if (!PtrTraits::IsValid(Z_ns.left_) && !PtrTraits::IsValid(Z_ns.right_))
                Z_ns.double_demote_rank();
            else
                Z_ns.demote_rank();
        } else {
            // Case #2: W is a 2-child of Y
            //
            // 1) Rotate-RL at V
            // 2) Rotate-LR at V
            // 3) Promote V twice.
            // 3) Demote Y once.
            // 3) Demote Z twice.
            RawPtrType V = LRTraits::LRRawChild(Y_ns);
            MX_DEBUG_ASSERT(PtrTraits::IsValid(V));                     // V must exist
            auto& V_ns = NodeTraits::node_state(*V);
            MX_DEBUG_ASSERT(V_ns.rank_parity() != Y_ns.rank_parity());  // V must be a 1-child of Y

            // TODO(johngro) : Special case the implementation of a double
            // rotation operation.  It would almost certainly be more efficient
            // than performing 2 sequential single rotation operations.
            RotateLR<RLTraits>(V, Y);
            RotateLR<LRTraits>(V, Z);
            Observer::RecordEraseDoubleRotation();

            V_ns.double_promote_rank();
            Y_ns.demote_rank();
            Z_ns.double_demote_rank();
        }
    }

    // Tree state consists of...
    //
    // 1) a managed root node
    // 2) an unmanaged left-most node
    // 3) an unmanaged right-most node
    // 4) a count of nodes
    //
    // Technically, only #1 is required.  #2-4 are optimizations to assist in
    // iteration and size operations.
    PtrType    root_       = nullptr;
    RawPtrType left_most_  = sentinel();
    RawPtrType right_most_ = sentinel();
    size_t     count_      = 0;
};

template <typename KeyType, typename PtrType, typename KeyTraits, typename NodeTraits, typename Obs>
constexpr bool WAVLTree<KeyType, PtrType, KeyTraits, NodeTraits, Obs>::SupportsConstantOrderErase;
template <typename KeyType, typename PtrType, typename KeyTraits, typename NodeTraits, typename Obs>
constexpr bool WAVLTree<KeyType, PtrType, KeyTraits, NodeTraits, Obs>::SupportsConstantOrderSize;
template <typename KeyType, typename PtrType, typename KeyTraits, typename NodeTraits, typename Obs>
constexpr bool WAVLTree<KeyType, PtrType, KeyTraits, NodeTraits, Obs>::IsAssociative;
template <typename KeyType, typename PtrType, typename KeyTraits, typename NodeTraits, typename Obs>
constexpr bool WAVLTree<KeyType, PtrType, KeyTraits, NodeTraits, Obs>::IsSequenced;

}  // namespace mxtl
