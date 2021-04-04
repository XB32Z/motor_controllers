// From
// https://stackoverflow.com/questions/27693812/how-to-handle-unique-ptrs-with-swig

namespace std {
  %feature("novaluewrapper") unique_ptr;
  template <typename Type, void(*deleter)(Type*)>
  struct unique_ptr {
     typedef Type* pointer;

     explicit unique_ptr( pointer Ptr );
     unique_ptr (unique_ptr&& Right);
     template<class Type2, Class Del2> unique_ptr( unique_ptr<Type2, Del2>&& Right );
     unique_ptr( const unique_ptr& Right) = delete;


     pointer operator-> () const;
     /*pointer release ();
     void reset (pointer __p=pointer());
     void swap (unique_ptr &__u);
     pointer get () const;*/
     operator bool () const;

     ~unique_ptr();
  };
}

%define wrap_unique_ptr(Name, Type, Deleter)
  %template(Name) std::unique_ptr<Type, Deleter>;
  //%newobject std::unique_ptr<Type, Deleter>::release;

  %typemap(out) std::unique_ptr<Type, Deleter> %{
    %set_output(SWIG_NewPointerObj((new $1_ltype(std::move($1))), $&1_descriptor,  $owner | SWIG_POINTER_OWN));
  %}

  %typemap(in) 
                std::unique_ptr<Type, Deleter>,
                std::unique_ptr<Type, Deleter>& %{
    {
      int newmem = 0;
      void * argp = NULL;
      int res = SWIG_ConvertPtrAndOwn($input, &argp,  $descriptor(std::unique_ptr< Type, Deleter > *), %convertptr_flags, &newmem);
      if (!SWIG_IsOK(res)) {
        %argument_fail(res, "$type", $symname, $argnum);
      }
      if (!argp) {
        %argument_nullref("$type", $symname, $argnum);
      } else {
        $1 = std::move(*(%reinterpret_cast(argp, $1_ltype*)));
        if (newmem & SWIG_CAST_NEW_MEMORY) delete %reinterpret_cast(argp, $1_ltype *);
      }
    }
  %}

  %typemap(typecheck) 
                      std::unique_ptr<Type, Deleter>,
                      std::unique_ptr<Type, Deleter>& %{
    int res = SWIG_ConvertPtr($input, 0, $descriptor(std::unique_ptr< Type, Deleter >), 0);
    $1 = SWIG_CheckState(res);
  %}

  %typemap(check) std::unique_ptr<Type, Deleter>& %{
    if($1 == 0)
      SWIG_fail;
  %}

%enddef