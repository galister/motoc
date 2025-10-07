#![allow(non_camel_case_types)]
#![allow(non_snake_case)]
#![allow(dead_code)]

use std::{ffi::CStr, mem::MaybeUninit, ptr};

use openxr as xr;

// change this to Arc if you need Sync + Send
type Xrc<T> = std::rc::Rc<T>;

#[derive(Clone)]
pub struct Mndx {
    pfn: Xrc<MndxPfn>,
}

struct MndxPfn {
    xr_create_xdev_list: PFN_xrCreateXDevListMNDX,
    xr_destroy_xdev_list: PFN_xrDestroyXDevListMNDX,
    xr_enumerate_xdevs: PFN_xrEnumerateXDevsMNDX,
    xr_get_xdev_properties: PFN_xrGetXDevPropertiesMNDX,
    xr_get_xdev_list_generation_number: PFN_xrGetXDevListGenerationNumberMNDX,
    xr_create_xdev_space: PFN_xrCreateXDevSpaceMNDX,
}

impl Mndx {
    pub fn new(instance: &xr::Instance) -> anyhow::Result<Self> {
        let create_xdev_list: PFN_xrCreateXDevListMNDX =
            load_pfn(instance, b"xrCreateXDevListMNDX\0")?;

        let destroy_xdev_list: PFN_xrDestroyXDevListMNDX =
            load_pfn(instance, b"xrDestroyXDevListMNDX\0")?;

        let enumerate_xdevs: PFN_xrEnumerateXDevsMNDX =
            load_pfn(instance, b"xrEnumerateXDevsMNDX\0")?;

        let get_xdev_properties: PFN_xrGetXDevPropertiesMNDX =
            load_pfn(instance, b"xrGetXDevPropertiesMNDX\0")?;

        let get_xdev_list_generation_number: PFN_xrGetXDevListGenerationNumberMNDX =
            load_pfn(instance, b"xrGetXDevListGenerationNumberMNDX\0")?;

        let create_xdev_space: PFN_xrCreateXDevSpaceMNDX =
            load_pfn(instance, b"xrCreateXDevSpaceMNDX\0")?;

        Ok(Self {
            pfn: Xrc::new(MndxPfn {
                xr_create_xdev_list: create_xdev_list,
                xr_destroy_xdev_list: destroy_xdev_list,
                xr_enumerate_xdevs: enumerate_xdevs,
                xr_get_xdev_properties: get_xdev_properties,
                xr_get_xdev_list_generation_number: get_xdev_list_generation_number,
                xr_create_xdev_space: create_xdev_space,
            }),
        })
    }

    pub fn create_list<G>(&self, session: &xr::Session<G>) -> anyhow::Result<XDevList> {
        let create_info = XrCreateXDevListInfoMNDX {
            type_: xr::sys::StructureType::from_raw(XR_TYPE_CREATE_XDEV_LIST_INFO_MNDX),
            next: ptr::null(),
        };

        let mut list: XrXDevListMNDX = unsafe { MaybeUninit::zeroed().assume_init() };

        unsafe {
            if ((self.pfn.xr_create_xdev_list.unwrap())(session.as_raw(), &create_info, &mut list))
                != xr::sys::Result::SUCCESS
            {
                anyhow::bail!("failed to create xdev list");
            }
        }

        Ok(XDevList {
            inner: Xrc::new(XDevListInner {
                id: list,
                mndx_pfn: self.pfn.clone(),
            }),
            mndx: self.clone(),
        })
    }
}

#[derive(Clone)]
pub struct XDevList {
    inner: Xrc<XDevListInner>,
    mndx: Mndx,
}

struct XDevListInner {
    id: XrXDevListMNDX,
    mndx_pfn: Xrc<MndxPfn>,
}

impl XDevList {
    pub fn enumerate_xdevs(&self) -> anyhow::Result<Vec<XDev>> {
        let mut raw_xdevs = Vec::with_capacity(64);
        let mut xdev_count: u32 = 0;
        unsafe {
            if (self.mndx.pfn.xr_enumerate_xdevs.unwrap())(
                self.inner.id,
                raw_xdevs.capacity() as _,
                &mut xdev_count,
                raw_xdevs.as_mut_ptr(),
            ) != xr::sys::Result::SUCCESS
            {
                anyhow::bail!("failed to enumerate xdevs");
            }
            raw_xdevs.set_len(xdev_count as _);
        }

        let mut xdevs = Vec::with_capacity(raw_xdevs.len());
        for d in raw_xdevs.into_iter() {
            xdevs.push(XDev::new(self.clone(), d)?);
        }
        Ok(xdevs)
    }

    pub fn get_generation_number(&self) -> anyhow::Result<u64> {
        let mut generation = 0;

        unsafe {
            if (self
                .inner
                .mndx_pfn
                .xr_get_xdev_list_generation_number
                .unwrap())(self.inner.id, &mut generation)
                != xr::sys::Result::SUCCESS
            {
                anyhow::bail!("failed to get generation number");
            }
        }

        Ok(generation)
    }
}

impl Drop for XDevListInner {
    fn drop(&mut self) {
        unsafe { (self.mndx_pfn.xr_destroy_xdev_list.unwrap())(self.id) };
    }
}

pub struct XDev {
    inner: Xrc<XDevInner>,
    list: XDevList,
}

impl XDev {
    pub fn id(&self) -> XrXDevIdMNDX {
        self.inner.id
    }
    pub fn name(&self) -> &str {
        &self.inner.name
    }
    pub fn serial(&self) -> &str {
        &self.inner.serial
    }
    pub fn can_create_space(&self) -> bool {
        self.inner.can_create_space
    }
}

struct XDevInner {
    id: XrXDevIdMNDX,
    name: String,
    serial: String,
    can_create_space: bool,
}

impl XDev {
    fn new(list: XDevList, id: XrXDevIdMNDX) -> anyhow::Result<Self> {
        let info = XrGetXDevInfoMNDX {
            type_: xr::sys::StructureType::from_raw(XR_TYPE_GET_XDEV_INFO_MNDX),
            next: ptr::null(),
            id: id as _,
        };

        let mut properties = XrXDevPropertiesMNDX {
            type_: xr::sys::StructureType::from_raw(XR_TYPE_XDEV_PROPERTIES_MNDX),
            next: ptr::null_mut(),
            name: [0; 256],
            serial: [0; 256],
            can_create_space: xr::sys::Bool32::from_raw(0),
        };

        unsafe {
            if (list.mndx.pfn.xr_get_xdev_properties.unwrap())(
                list.inner.id,
                &info,
                &mut properties,
            ) != xr::sys::Result::SUCCESS
            {
                anyhow::bail!("Failed to get XDEV properties");
            }
        }

        Ok(XDev {
            inner: Xrc::new(XDevInner {
                id,
                name: CStr::from_bytes_until_nul(&properties.name)?
                    .to_string_lossy()
                    .into(),
                serial: CStr::from_bytes_until_nul(&properties.serial)?
                    .to_string_lossy()
                    .into(),
                can_create_space: properties.can_create_space.into(),
            }),
            list,
        })
    }

    pub fn create_space<G>(&self, session: xr::Session<G>) -> anyhow::Result<xr::Space> {
        let create_info = XrCreateXDevSpaceInfoMNDX {
            type_: xr::sys::StructureType::from_raw(XR_TYPE_CREATE_XDEV_SPACE_INFO_MNDX),
            next: ptr::null(),
            xdev_list: self.list.inner.id,
            id: self.inner.id,
            offset: xr::Posef::IDENTITY,
        };

        let mut space: xr::sys::Space = unsafe { MaybeUninit::zeroed().assume_init() };

        unsafe {
            if (self.list.mndx.pfn.xr_create_xdev_space.unwrap())(
                session.as_raw(),
                &create_info,
                &mut space,
            ) != xr::sys::Result::SUCCESS
            {
                anyhow::bail!("Failed to create XDEV space.");
            }
        }

        Ok(unsafe { xr::Space::reference_from_raw(session, space) })
    }
}

pub const XDEV_SPACE_EXTENSION_NAME: &str = "XR_MNDX_xdev_space";

pub type XrXDevIdMNDX = u64;

#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct XrXDevListMNDX_T {
    _unused: [u8; 0],
}
pub type XrXDevListMNDX = *mut XrXDevListMNDX_T;

const XR_TYPE_SYSTEM_XDEV_SPACE_PROPERTIES_MNDX: i32 = 1000444001;

#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct XrSystemXDevSpacePropertiesMNDX {
    pub type_: xr::sys::StructureType,
    pub next: *mut ::std::os::raw::c_void,
    pub supports_xdev_space: xr::sys::Bool32,
}
const XR_TYPE_CREATE_XDEV_LIST_INFO_MNDX: i32 = 1000444002;

#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct XrCreateXDevListInfoMNDX {
    pub type_: xr::sys::StructureType,
    pub next: *const ::std::os::raw::c_void,
}

const XR_TYPE_GET_XDEV_INFO_MNDX: i32 = 1000444003;
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct XrGetXDevInfoMNDX {
    pub type_: xr::sys::StructureType,
    pub next: *const ::std::os::raw::c_void,
    pub id: XrXDevIdMNDX,
}

const XR_TYPE_XDEV_PROPERTIES_MNDX: i32 = 1000444004;
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct XrXDevPropertiesMNDX {
    pub type_: xr::sys::StructureType,
    pub next: *mut ::std::os::raw::c_void,
    pub name: [u8; 256],
    pub serial: [u8; 256],
    pub can_create_space: xr::sys::Bool32,
}
#[allow(clippy::unnecessary_operation, clippy::identity_op)]
const XR_TYPE_CREATE_XDEV_SPACE_INFO_MNDX: i32 = 1000444005;
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct XrCreateXDevSpaceInfoMNDX {
    pub type_: xr::sys::StructureType,
    pub next: *const ::std::os::raw::c_void,
    pub xdev_list: XrXDevListMNDX,
    pub id: XrXDevIdMNDX,
    pub offset: xr::Posef,
}

pub type PFN_xrCreateXDevListMNDX = ::std::option::Option<
    unsafe extern "C" fn(
        session: xr::sys::Session,
        info: *const XrCreateXDevListInfoMNDX,
        xdevList: *mut XrXDevListMNDX,
    ) -> xr::sys::Result,
>;
pub type PFN_xrGetXDevListGenerationNumberMNDX = ::std::option::Option<
    unsafe extern "C" fn(xdevList: XrXDevListMNDX, outGeneration: *mut u64) -> xr::sys::Result,
>;
pub type PFN_xrEnumerateXDevsMNDX = ::std::option::Option<
    unsafe extern "C" fn(
        xdevList: XrXDevListMNDX,
        xdevCapacityInput: u32,
        xdevCountOutput: *mut u32,
        xdevs: *mut XrXDevIdMNDX,
    ) -> xr::sys::Result,
>;
pub type PFN_xrGetXDevPropertiesMNDX = ::std::option::Option<
    unsafe extern "C" fn(
        xdevList: XrXDevListMNDX,
        info: *const XrGetXDevInfoMNDX,
        properties: *mut XrXDevPropertiesMNDX,
    ) -> xr::sys::Result,
>;
pub type PFN_xrDestroyXDevListMNDX =
    ::std::option::Option<unsafe extern "C" fn(xdevList: XrXDevListMNDX) -> xr::sys::Result>;

pub type PFN_xrCreateXDevSpaceMNDX = ::std::option::Option<
    unsafe extern "C" fn(
        session: xr::sys::Session,
        createInfo: *const XrCreateXDevSpaceInfoMNDX,
        space: *mut xr::sys::Space,
    ) -> xr::sys::Result,
>;

fn load_pfn<T>(instance: &xr::Instance, name: &[u8]) -> anyhow::Result<Option<T>> {
    let mut result: Option<T> = None;
    unsafe {
        if (instance.fp().get_instance_proc_addr)(
            instance.as_raw(),
            name.as_ptr() as _,
            &mut result as *mut Option<T> as *mut _,
        ) != xr::sys::Result::SUCCESS
        {
            anyhow::bail!("Could not load {:?}!", name);
        }
    }
    Ok(result)
}
