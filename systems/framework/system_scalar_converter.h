#pragma once

#include <memory>
#include <sstream>
#include <typeindex>
#include <typeinfo>
#include <unordered_map>
#include <utility>

#include "drake/common/autodiff.h"
#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/drake_deprecated.h"
#include "drake/common/nice_type_name.h"
#include "drake/common/symbolic.h"
#include "drake/systems/framework/scalar_conversion_traits.h"
#include "drake/systems/framework/system_type_tag.h"

namespace drake {
namespace systems {

template <typename T> class System;

/// Helper class to convert a System<U> into a System<T>, intended for internal
/// use by the System framework, not directly by users.
///
/// For user-facing documentation see @ref system_scalar_conversion.
///
/// Because it is not templated on a System subclass, this class can be used by
/// LeafSystem without any direct knowledge of the subtypes being converted.
/// In other words, it enables a runtime flavor of the CRTP.
///
/// Throughout this class, the following template naming convention applies:
///
/// @tparam S is the System subclass that this object will convert from and to.
/// @tparam U the source scalar type (to convert from), which must be one of
///         the @ref default_scalars "default scalars".
/// @tparam T the resulting scalar type (to convert into), which must be one of
///         the @ref default_scalars "default scalars".
///
/// @note Conversions where `T` and `U` type are the same are not supported.
/// Template functions such as IsConvertible<T, U>() are still callable, but
/// will always return false, null, etc.
class SystemScalarConverter {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(SystemScalarConverter);

  /// (Advanced) Creates a converter that supports no conversions.  The single-
  /// argument constructor below is the overload intended for users.
  SystemScalarConverter();

  /// Creates a converter that uses S's scalar-converting copy constructor to
  /// perform system scalar conversion. That constructor takes the form of:
  ///
  /// @code
  /// template <typename T>
  /// class FooSystem final : public LeafSystem<T> {
  ///   template <typename U>
  ///   explicit FooSystem(const FooSystem<U>& other);
  /// };
  /// @endcode
  ///
  /// By default, the converter supports conversions to and from all of the
  /// @ref default_scalars "default scalars", but systems may specialize the
  /// scalar_conversion::Traits to disable support for some or all of these
  /// conversions.
  ///
  /// This an implicit conversion constructor (not marked `explicit`), in order
  /// to make calling code substantially more readable, with relatively little
  /// risk of an unwanted accidental conversion happening.
  ///
  /// See @ref system_scalar_conversion for additional overview documentation.
  template <template <typename> class S>
  // NOLINTNEXTLINE(runtime/explicit)
  SystemScalarConverter(SystemTypeTag<S> tag)
      : SystemScalarConverter(tag, GuaranteedSubtypePreservation::kEnabled) {}

  /// A configuration option for our constructor, controlling whether or not
  /// the Convert implementation requires that the System subclass type is
  /// preserved.
  enum class GuaranteedSubtypePreservation {
    /// The argument to Convert must be of the exact type S that was used to
    /// populate the SystemScalarConverter.
    kEnabled,
    /// The argument to Convert need not be the exact type S that was used to
    /// populate the SystemScalarConverter -- it can be either exactly that S,
    /// or a subtype of that S.  This permits subtype information to be lost
    /// across conversion.
    kDisabled,
  };

  /// (Advanced)  Creates using S's scalar-type converting copy constructor.
  /// Behaves exactly like SystemScalarConverter(SystemTypeTag<S>), but with
  /// the additional option to turn off guaranteed subtype preservation of the
  /// System being converted.  In general, subtype preservation is an important
  /// invariant during scalar conversion, so be cautious about disabling it.
  template <template <typename> class S>
  SystemScalarConverter(
      SystemTypeTag<S>, GuaranteedSubtypePreservation subtype_preservation)
      : SystemScalarConverter() {
    // N.B. When changing the pairs of supported types below, be sure to also
    // change the `ConversionPairs` type pack in `DefineFrameworkPySystems`.
    using Expression = symbolic::Expression;
    // From double to all other types.
    AddIfSupported<S, AutoDiffXd, double>(subtype_preservation);
    AddIfSupported<S, Expression, double>(subtype_preservation);
    // From AutoDiffXd to all other types.
    AddIfSupported<S, double,     AutoDiffXd>(subtype_preservation);
    AddIfSupported<S, Expression, AutoDiffXd>(subtype_preservation);
    // From Expression to all other types.
    AddIfSupported<S, double,     Expression>(subtype_preservation);
    AddIfSupported<S, AutoDiffXd, Expression>(subtype_preservation);
  }

  /// Returns true iff no conversions are supported.  (In other words, whether
  /// this is a default-constructed object.)
  bool empty() const { return funcs_.empty(); }

  template <typename T, typename U>
  using ConverterFunction
      DRAKE_DEPRECATED("2021-10-01",
      "Only scalar-converting copy constructors are supported.")
      = std::function<std::unique_ptr<System<T>>(const System<U>&)>;

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  template <typename T, typename U>
  DRAKE_DEPRECATED("2021-10-01",
      "Only scalar-converting copy constructors are supported.")
  void Add(const ConverterFunction<T, U>&);
#pragma GCC diagnostic pop

  template <template <typename> class S, typename T, typename U>
  DRAKE_DEPRECATED("2021-10-01",
      "User-defined scalar types cannot be added.")
  void AddIfSupported() {
    AddIfSupported<S, T, U>(GuaranteedSubtypePreservation::kEnabled);
  }

  /// Removes from this converter all pairs where `other.IsConvertible<T, U>`
  /// is false.  The subtype `S` need not be the same between this and `other`.
  void RemoveUnlessAlsoSupportedBy(const SystemScalarConverter& other);

  /// Removes from this converter the ability to convert from System<U> to
  /// System<T>.
  template <typename T, typename U>
  void Remove();

  /// Returns true iff this object can convert a System<U> into a System<T>,
  /// i.e., whether Convert() will return non-null.
  template <typename T, typename U>
  bool IsConvertible() const;

  /// Converts a System<U> into a System<T>.  This is the API that LeafSystem
  /// uses to provide a default implementation of DoToAutoDiffXd, etc.
  template <typename T, typename U>
  std::unique_ptr<System<T>> Convert(const System<U>& other) const;

 private:
  // Like ConverterFunc, but with the args and return value decayed into void*.
  using ErasedConverterFunc = std::function<void*(const void*)>;

  // A pair of types {T, U}, usable as an unordered_map key.
  struct Key : std::pair<std::type_index, std::type_index> {
    Key(const std::type_info&, const std::type_info&);
  };
  struct KeyHasher {
    size_t operator()(const Key&) const;
  };

  // An overload of AddIfSupported that offers to disable subtype preservation.
  template <template <typename> class S, typename T, typename U>
  void AddIfSupported(GuaranteedSubtypePreservation subtype_preservation);

  // Given typeid(T), typeid(U), returns a converter.  If no converter has been
  // added yet, returns nullptr.
  const ErasedConverterFunc* Find(
      const std::type_info&, const std::type_info&) const;

  // Given typeid(T), typeid(U), adds a converter.
  void Insert(
      const std::type_info&, const std::type_info&,
      const ErasedConverterFunc&);

  // Maps from {T, U} to the function that converts from U into T.
  std::unordered_map<Key, ErasedConverterFunc, KeyHasher> funcs_;
};

#if !defined(DRAKE_DOXYGEN_CXX)

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
// (This function is deprecated.)
template <typename T, typename U>
void SystemScalarConverter::Add(const ConverterFunction<T, U>& func) {
  // Make sure func contains a target (i.e., is not null-ish).
  DRAKE_ASSERT(static_cast<bool>(func));
  // Copy `func` into a lambda that ends up stored into `funcs_`.  The lambda
  // is typed as `void* => void*` in order to have a non-templated signature
  // and thus fit into a homogeneously-typed std::unordered_map.
  Insert(typeid(T), typeid(U), [func](const void* const bare_u) {
    DRAKE_ASSERT(bare_u);
    const System<U>& other = *static_cast<const System<U>*>(bare_u);
    return func(other).release();
  });
}
#pragma GCC diagnostic pop

template <typename T, typename U>
bool SystemScalarConverter::IsConvertible() const {
  const ErasedConverterFunc* converter = Find(typeid(T), typeid(U));
  return (converter != nullptr);
}

template <typename T, typename U>
std::unique_ptr<System<T>> SystemScalarConverter::Convert(
    const System<U>& other) const {
  // Lookup the lambda that Add() stored and call it.
  System<T>* result = nullptr;
  const ErasedConverterFunc* converter = Find(typeid(T), typeid(U));
  if (converter) {
    result = static_cast<System<T>*>((*converter)(&other));
  }
  return std::unique_ptr<System<T>>(result);
}

namespace system_scalar_converter_internal {
// When Traits says that conversion is supported.
// N.B. This logic should be reflected in `TemplateSystem._make` in the file
// `scalar_conversion.py`.
template <template <typename> class S, typename T, typename U>
static std::unique_ptr<System<T>> Make(
    bool subtype_preservation, const System<U>& other, std::true_type) {
  // We conditionally require that system scalar conversion maintain the exact
  // system type.  Fail fast if `other` is not of exact type S<U>.
  if (subtype_preservation &&
      (std::type_index{typeid(other)} != std::type_index{typeid(S<U>)})) {
    std::ostringstream msg;
    msg << "SystemScalarConverter::Convert was configured to convert a "
        << NiceTypeName::Get<S<U>>() << " into a "
        << NiceTypeName::Get<S<T>>() << " but was called with a "
        << NiceTypeName::Get(other) << " at runtime";
    throw std::runtime_error(msg.str());
  }
  const S<U>& my_other = dynamic_cast<const S<U>&>(other);
  auto result = std::make_unique<S<T>>(my_other);
  // We manually propagate the name from the old System to the new.  The name
  // is the only extrinsic property of the System and LeafSystem base classes
  // that is stored within the System itself.
  result->set_name(other.get_name());
  return result;
}
// When Traits says not to convert.
template <template <typename> class S, typename T, typename U>
static std::unique_ptr<System<T>> Make(
    bool, const System<U>&, std::false_type) {
  // AddIfSupported is guaranteed not to call us, but we *will* be compiled,
  // so we have to have some kind of function body.
  throw std::logic_error("system_scalar_converter_internal");
}
}  // namespace system_scalar_converter_internal

template <template <typename> class S, typename T, typename U>
void SystemScalarConverter::AddIfSupported(
    GuaranteedSubtypePreservation subtype_preservation) {
  using supported =
      typename scalar_conversion::Traits<S>::template supported<T, U>;
  if (supported::value) {
    // The lambda is typed as `void* => void*` in order to have a non-templated
    // signature and thus fit into a homogeneously-typed std::unordered_map.
    auto func = [subtype_preservation](const void* const other_system_u)
        -> void* {
      DRAKE_ASSERT(other_system_u != nullptr);
      const System<U>& other = *static_cast<const System<U>*>(other_system_u);
      // Dispatch to an overload based on whether S<U> ==> S<T> is supported.
      // (At runtime, this block is only executed for supported conversions,
      // but at compile time, Make will be instantiated unconditionally.)
      std::unique_ptr<System<T>> result =
          system_scalar_converter_internal::Make<S, T, U>(
              (subtype_preservation == GuaranteedSubtypePreservation::kEnabled),
              other, supported{});
      return result.release();
    };
    Insert(typeid(T), typeid(U), func);
  }
}

#endif  // DRAKE_DOXYGEN_CXX

}  // namespace systems
}  // namespace drake
