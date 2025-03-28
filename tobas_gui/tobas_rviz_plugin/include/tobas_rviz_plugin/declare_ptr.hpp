#pragma once

#include <memory>

/**
 * \def TOBAS_DELCARE_PTR
 * Macro that given a Name and a Type declares the following types:
 * - ${Name}Ptr            = shared_ptr<${Type}>
 * - ${Name}ConstPtr       = shared_ptr<const ${Type}>
 * - ${Name}WeakPtr        = weak_ptr<${Type}>
 * - ${Name}ConstWeakPtr   = weak_ptr<const ${Type}>
 * - ${Name}UniquePtr      = unique_ptr<${Type}>
 * - ${Name}ConstUniquePtr = unique_ptr<const ${Type}>
 *
 * For best portability the exact type of shared_ptr declared by the macro
 * should be considered to be an implementation detail, liable to change in
 * future releases.
 */
#define TOBAS_DECLARE_PTR(Name, Type)                                                                                  \
  typedef std::shared_ptr<Type> Name##Ptr;                                                                             \
  typedef std::shared_ptr<const Type> Name##ConstPtr;                                                                  \
  typedef std::weak_ptr<Type> Name##WeakPtr;                                                                           \
  typedef std::weak_ptr<const Type> Name##ConstWeakPtr;                                                                \
  typedef std::unique_ptr<Type> Name##UniquePtr;                                                                       \
  typedef std::unique_ptr<const Type> Name##ConstUniquePtr

/**
 * \def TOBAS_DELCARE_PTR_MEMBER
 * The macro defines the same typedefs as TOBAS_DECLARE_PTR, but shortens the new names to their suffix.
 *
 * This can be used to create `Classname::Ptr` style names, but in most situations in Tobas's codebase,
 * TOBAS_CLASS_FORWARD and TOBAS_DECLARE_PTR should be preferred.
 */
#define TOBAS_DECLARE_PTR_MEMBER(Type)                                                                                 \
  typedef std::shared_ptr<Type> Ptr;                                                                                   \
  typedef std::shared_ptr<const Type> ConstPtr;                                                                        \
  typedef std::weak_ptr<Type> WeakPtr;                                                                                 \
  typedef std::weak_ptr<const Type> ConstWeakPtr;                                                                      \
  typedef std::unique_ptr<Type> UniquePtr;                                                                             \
  typedef std::unique_ptr<const Type> ConstUniquePtr
